#include <thread>
#include <deque>
#include <queue>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf_conversions/tf_eigen.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "utils/tfTools.hpp"
#include "utils/common_names.h"
#include "utils/stampTools.hpp"

using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::P;
using symbol_shorthand::V;

class ImuOdometry
{
    struct StampedNavState
    {
        ros::Time stamp;
        NavState state;
        imuBias::ConstantBias bias;
    }; // struct StampedNavState

    struct GraphState
    {
        using cov_ptr = noiseModel::Base::shared_ptr;
        GraphState(){}
        GraphState(const ros::Time& stamp_, const NavState& state_, const imuBias::ConstantBias& bias_, 
        const cov_ptr& pose_cov_ = nullptr, const cov_ptr& velocity_cov_ = nullptr, const cov_ptr& bias_cov_ = nullptr)
        : stamp(stamp_), state(state_), bias(bias_), pose_cov(pose_cov_), velocity_cov(velocity_cov_), bias_cov(bias_cov_){}

        ros::Time stamp;
        NavState state;
        imuBias::ConstantBias bias;

        cov_ptr pose_cov, velocity_cov, bias_cov;
    }; // struct GraphState

public:
    ImuOdometry()
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        P_IMU_BASE = read_static_tf(imu_link_frame_id, base_link_frame_id);
        const auto P_lidar_imu = read_static_tf(lidar_link_frame_id, imu_link_frame_id);
        const auto P_base_imu = read_static_tf(base_link_frame_id, imu_link_frame_id);
        _is_odometer = (imu_link_frame_id == nh.param<std::string>("odometer_link", ""));
        double prior_velocity_noise = private_nh.param<double>("prior_velocity_noise", 1);
        double prior_acc_bias_noise = private_nh.param<double>("prior_acc_bias_noise", 0.05);
        double prior_gyro_bias_noise = private_nh.param<double>("prior_gyro_bias_noise", 0.002);
        double prior_roll_pitch_noise = private_nh.param<double>("prior_roll_pitch_noise", 0.05);
        double lidar_translation_inc_noise = private_nh.param<double>("lidar_translation_inc_noise", 0.001);
        double lidar_rotation_inc_noise = private_nh.param<double>("lidar_rotation_inc_noise", M_PI / 180 * 0.2);
        double acc_noise = private_nh.param<double>("acc_noise", 0.1);
        double gyro_noise = private_nh.param<double>("gyro_noise", 0.01);
        double integration_noise = private_nh.param<double>("integration_noise", 0.0001);
        double acc_drift_per_sec = private_nh.param<double>("acc_drift_per_sec", 0.01);
        double gyro_drift_per_sec = private_nh.param<double>("gyro_drift_per_sec", 0.00017);
        double gravity = private_nh.param<double>("gravity", 9.8099);
        RESET_PERIOD = private_nh.param<double>("reset_period", 8);
        _rotation_calibrated = !private_nh.param<bool>("re_calibrate_gravity", true);

        Eigen::Isometry3d T_lidar_imu, T_base_imu;
        tf::poseTFToEigen(P_lidar_imu, T_lidar_imu);
        tf::poseTFToEigen(P_base_imu, T_base_imu);
        T_LIDAR_IMU = Pose3(T_lidar_imu.matrix());

        HUBER = noiseModel::mEstimator::Huber::Create(0.1);

        PRIOR_VELOCITY = Vector3::Zero();
        PRIOR_BIAS = imuBias::ConstantBias();

        PRIOR_VELOCITY_NOISE = noiseModel::Robust::Create(HUBER, noiseModel::Isotropic::Sigma(3, prior_velocity_noise));
        PRIOR_BIAS_NOISE = noiseModel::Robust::Create(HUBER, noiseModel::Diagonal::Sigmas((Vector6() << prior_acc_bias_noise, prior_acc_bias_noise, prior_acc_bias_noise, prior_gyro_bias_noise, prior_gyro_bias_noise, prior_gyro_bias_noise).finished()));
        PRIOR_POSE_NOISE = noiseModel::Robust::Create(HUBER, noiseModel::Diagonal::Sigmas((Vector6() << prior_roll_pitch_noise, prior_roll_pitch_noise, 0.001, 0.001, 0.001, 0.001).finished())); 
        PRIOR_LIDAR_POSE_INC_NOISE = noiseModel::Robust::Create(HUBER, noiseModel::Diagonal::Sigmas(T_LIDAR_IMU.inverse().AdjointMap() * (Vector6() << lidar_rotation_inc_noise, lidar_rotation_inc_noise, lidar_rotation_inc_noise, lidar_translation_inc_noise, lidar_translation_inc_noise, lidar_translation_inc_noise).finished()));

        IMU_ACC_PARAMS = PreintegrationParams::MakeSharedU(gravity);
        IMU_ACC_PARAMS->setAccelerometerCovariance(Matrix3::Identity() * acc_noise * acc_noise);
        IMU_ACC_PARAMS->setGyroscopeCovariance(Matrix3::Identity() * gyro_noise * gyro_noise);
        IMU_ACC_PARAMS->setIntegrationCovariance(Matrix3::Identity() * integration_noise * integration_noise);
        BIAS_PER_SEC << acc_drift_per_sec, acc_drift_per_sec, acc_drift_per_sec, gyro_drift_per_sec, gyro_drift_per_sec, gyro_drift_per_sec;

        _pre_nav_state.bias = imuBias::ConstantBias();
        _pre_nav_state.state = NavState(Pose3(T_base_imu.matrix()), Vector3::Zero());
        _imu_accumulator = boost::make_shared<PreintegratedImuMeasurements>(IMU_ACC_PARAMS, _pre_nav_state.bias);
        _imu_accumulator_opt = boost::make_shared<PreintegratedImuMeasurements>(IMU_ACC_PARAMS, _pre_nav_state.bias);

        ISAM_PARAMS.setRelinearizeSkip(1);
        ISAM_PARAMS.setRelinearizeThreshold(0.01);

        _pub_pose = nh.advertise<geometry_msgs::PoseStamped>(topic_odometer + std::string("/") + imu_link_frame_id, 16);
        _pub_twist = nh.advertise<geometry_msgs::TwistStamped>(topic_velometer + std::string("/") + imu_link_frame_id, 16);
        _sub_imu = nh.subscribe<sensor_msgs::Imu>(topic_imu_msg, 4096, &ImuOdometry::_imu_handler, this);

        _optimize_thread = std::thread(&ImuOdometry::_optimize_process, this);
    }

    void SpinOnMultiQueue()
    {
        ros::NodeHandle nh;
        ros::CallbackQueue queue;
        nh.setCallbackQueue(&queue);
        _sub_odom = nh.subscribe<geometry_msgs::PoseStamped>(topic_odometry_lidar_optimized, 8, &ImuOdometry::_odom_handler, this);
        ros::AsyncSpinner aspinner(1, &queue);
        aspinner.start();
        ros::spin();
    }

    ~ImuOdometry()
    {
        _optimize_thread.join();
    }

private:    
    StampedNavState _initialize_graph(uint64_t init_key)
    {
        _isam = boost::make_shared<IncrementalFixedLagSmoother>(RESET_PERIOD, ISAM_PARAMS);
        StampedNavState graph_state;

        _m_imu_buf.lock();
        graph_state.stamp = _pre_nav_state.stamp;
        graph_state.state = _pre_nav_state.state;
        graph_state.bias = _pre_nav_state.bias;
        _m_imu_buf.unlock();

        NonlinearFactorGraph graph;
        Values initial_guess;
        FixedLagSmoother::KeyTimestampMap map_key_time;
        
        graph.addPrior(P(init_key), graph_state.state.pose(), PRIOR_POSE_NOISE);

        graph.addPrior(V(init_key), graph_state.state.velocity(), PRIOR_VELOCITY_NOISE);

        graph.addPrior(B(init_key), graph_state.bias, PRIOR_BIAS_NOISE);

        initial_guess.insert(P(init_key), graph_state.state.pose());
        initial_guess.insert(V(init_key), graph_state.state.velocity());
        initial_guess.insert(B(init_key), graph_state.bias);        

        map_key_time[P(init_key)] = map_key_time[V(init_key)] = map_key_time[B(init_key)] = graph_state.stamp.toSec();

        _isam->update(graph, initial_guess, map_key_time);
        _isam->update();

        return graph_state;  
    }

    void _optimize_process()
    {
        ros::NodeHandle nh;
        ros::Rate rate(100);
        ros::Time nouse;

        while(nh.ok())
        {
            _m_imu_buf.lock();
            if(_pre_nav_state.stamp.is_zero())
            {
                _m_imu_buf.unlock();
                rate.sleep();
                continue;
            }
            _m_imu_buf.unlock();

            uint64_t key = 0;
            auto graph_state = _initialize_graph(key++);
            wait_and_pop(_imu_buf_opt, graph_state.stamp, 100, nouse, nh);
            wait_and_pop_with_lock(_odom_buf, _m_odom_buf, graph_state.stamp, 100, nouse, nh);

            _imu_accumulator_opt->resetIntegrationAndSetBias(graph_state.bias);

            NonlinearFactorGraph graph;
            Values initial_guess;
            FixedLagSmoother::KeyTimestampMap map_key_time;

            while (nh.ok())
            {
                _wait_for_imu();
                const auto odom_msg = _pop_next_odom_msg();
                Pose3 pose_inc;;
                _calc_pose_increase(odom_msg, pose_inc);

                _integrate_by_range(*_imu_accumulator_opt, graph_state.stamp, ROS_TIME(odom_msg));

                const auto predict_state = _imu_accumulator_opt->predict(graph_state.state, graph_state.bias);

                graph.resize(0);
                initial_guess.clear();
                map_key_time.clear();

                map_key_time[P(key)] = map_key_time[V(key)] = map_key_time[B(key)] = ROS_TIME(odom_msg).toSec();

                initial_guess.insert(P(key), predict_state.pose());
                initial_guess.insert(V(key), predict_state.velocity());
                initial_guess.insert(B(key), graph_state.bias);

                graph.emplace_shared<ImuFactor>(P(key - 1), V(key - 1), P(key), V(key), B(key - 1), *_imu_accumulator_opt);
                graph.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(B(key - 1), B(key), imuBias::ConstantBias(),
                                     noiseModel::Robust::Create(HUBER, noiseModel::Diagonal::Sigmas(std::sqrt(_imu_accumulator_opt->deltaTij()) * BIAS_PER_SEC)));
      
                if(key > 1)
                    graph.emplace_shared<BetweenFactor<Pose3>>(P(key-1), P(key), pose_inc, noiseModel::Robust::Create(HUBER, PRIOR_LIDAR_POSE_INC_NOISE));
               
                _isam->update(graph, initial_guess, map_key_time);
                _isam->update();
                _isam->update();

                const auto &result = _isam->calculateEstimate();
                const auto &opt_pose = result.at(P(key)).cast<Pose3>();
                const auto &opt_velocity = result.at(V(key)).cast<Vector3>();
                const auto &opt_bias = result.at(B(key)).cast<imuBias::ConstantBias>();

                Matrix pose_cov = _isam->marginalCovariance(P(key));
                // soft reset
                if(std::sqrt(pose_cov(3,3) + pose_cov(4,4) + pose_cov(5,5)) > 15)
                {
                    graph.resize(0);
                    graph.emplace_shared<PriorFactor<Pose3>>(P(key), opt_pose, PRIOR_POSE_NOISE);
                    _isam->update(graph);
                }

                graph_state.stamp = ROS_TIME(odom_msg);
                graph_state.state = NavState(opt_pose, opt_velocity);
                graph_state.bias = opt_bias;
                _imu_accumulator_opt->resetIntegrationAndSetBias(graph_state.bias);

                ++key;

                if(key <= 2) continue;

                std::lock_guard<std::mutex> lock_guard(_m_imu_buf);
                ros::Time pre_stamp = graph_state.stamp;
                while (!_imu_buf_odom.empty() && ROS_TIME(_imu_buf_odom.front()) <= pre_stamp)
                    _imu_buf_odom.pop_front();
                for (const auto &imu_msg : _imu_buf_odom)
                {
                    _imu_accumulator_opt->integrateMeasurement(Vector3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z),
                                                Vector3(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z),
                                                (ROS_TIME(imu_msg) - pre_stamp).toSec());
                    pre_stamp = imu_msg->header.stamp;
                }
                const auto nav_state = _imu_accumulator_opt->predict(graph_state.state, graph_state.bias);
                const auto odom_state = _imu_accumulator->predict(_pre_nav_state.state, _pre_nav_state.bias);
                const auto opt_gravaty = odom_state.attitude().rotate(nav_state.attitude().unrotate(_imu_accumulator_opt->params()->getGravity()));

                auto new_params = boost::make_shared<PreintegrationParams>(opt_gravaty); 
                new_params->setAccelerometerCovariance(IMU_ACC_PARAMS->getAccelerometerCovariance());
                new_params->setGyroscopeCovariance(IMU_ACC_PARAMS->getGyroscopeCovariance());
                new_params->setIntegrationCovariance(IMU_ACC_PARAMS->getIntegrationCovariance());
                _imu_accumulator = boost::make_shared<PreintegratedImuMeasurements>(new_params, graph_state.bias);
                _pre_nav_state.stamp = pre_stamp;
                _pre_nav_state.bias = graph_state.bias;
                _pre_nav_state.state = NavState(odom_state.pose(), odom_state.R() * nav_state.bodyVelocity());
                _imu_accumulator_opt->resetIntegrationAndSetBias(graph_state.bias);
            }
        }
    }

    void _pub_imu_odometry(const Pose3 &odom_imu, const Vector6 &twist_imu, const ros::Time &stamp)
    {
        static tf::TransformBroadcaster br;
        static geometry_msgs::PoseStamped pose_msg;
        static geometry_msgs::TwistStamped twist_msg;
        static tf::StampedTransform T_odom_base;
        pose_msg.header.frame_id = odom_frame_id;
        twist_msg.header.frame_id = base_link_frame_id;

        const auto &t = odom_imu.translation();
        const auto &q = odom_imu.rotation().toQuaternion();
        pose_msg.header.stamp = stamp;
        pose_msg.pose.position.x = t.x();
        pose_msg.pose.position.y = t.y();
        pose_msg.pose.position.z = t.z();
        pose_msg.pose.orientation.w = q.w();
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        _pub_pose.publish(pose_msg);

        twist_msg.header.stamp = stamp;
        twist_msg.twist.angular.x = twist_imu[0];
        twist_msg.twist.angular.y = twist_imu[1];
        twist_msg.twist.angular.z = twist_imu[2];
        twist_msg.twist.linear.x = twist_imu[3];
        twist_msg.twist.linear.y = twist_imu[4];
        twist_msg.twist.linear.z = twist_imu[5];
        _pub_twist.publish(twist_msg);

        if(!_is_odometer) return;
        T_odom_base.setOrigin(tf::Vector3(t.x(), t.y(), t.z()));
        T_odom_base.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        T_odom_base.setData(T_odom_base * P_IMU_BASE);
        T_odom_base.frame_id_ = odom_frame_id;
        T_odom_base.child_frame_id_ = base_link_frame_id;
        T_odom_base.stamp_ = stamp;
        br.sendTransform(T_odom_base);
    }


    void _imu_handler(const sensor_msgs::ImuConstPtr &imu_msg)
    {
        if(!_rotation_calibrated)
        {
            static int count = 0;
            static gtsam::Point3 acc = gtsam::Point3::Zero();
            
            if(count >= 100)
            {
                acc /= count;
                const auto g1 = _pre_nav_state.state.attitude().rotate(acc).normalized();
                const Point3 addition_rot = g1.cross(Point3::UnitZ()).normalized() * std::acos(g1.dot(Point3::UnitZ()));
                acc.print("acc:\n");
                addition_rot.print("additon rot:\n");
                _pre_nav_state.state = NavState(Pose3(Rot3::AxisAngle(addition_rot.normalized(), addition_rot.norm()), Point3::Zero())  * _pre_nav_state.state.pose(), _pre_nav_state.state.velocity());
                _rotation_calibrated = true;
                return;
            }

            acc[0] += imu_msg->linear_acceleration.x;
            acc[1] += imu_msg->linear_acceleration.y;
            acc[2] += imu_msg->linear_acceleration.z;
            ++count;
            return;
        }

        _m_imu_buf.lock();

        static ros::Time last_stamp(0);
        if (last_stamp.isZero())
        {
            last_stamp = ROS_TIME(imu_msg);
            _pre_nav_state.stamp = last_stamp;
            _m_imu_buf.unlock();
            return;
        }

        if(imu_msg->header.stamp <= last_stamp) 
        {
            _m_imu_buf.unlock();
            return;
        } 

        _imu_buf_odom.push_back(imu_msg);
        _imu_buf_opt.push_back(imu_msg);

        double dt = (imu_msg->header.stamp - last_stamp).toSec();
        last_stamp = imu_msg->header.stamp;

        Vector3 linear_acc(imu_msg->linear_acceleration.x,
                           imu_msg->linear_acceleration.y,
                           imu_msg->linear_acceleration.z);

        Vector3 angular_rate(imu_msg->angular_velocity.x,
                             imu_msg->angular_velocity.y,
                             imu_msg->angular_velocity.z);

        _imu_accumulator->integrateMeasurement(linear_acc, angular_rate, dt);

        const auto cur_nav_state = _imu_accumulator->predict(_pre_nav_state.state, _pre_nav_state.bias);

        Pose3 imu_odom(cur_nav_state.quaternion(), cur_nav_state.position());
        Vector6 imu_twist = (Vector6() << angular_rate - _pre_nav_state.bias.gyroscope(), cur_nav_state.bodyVelocity()).finished();

        _m_imu_buf.unlock();

        _pub_imu_odometry(imu_odom, imu_twist, imu_msg->header.stamp);
    }

    /*
        lidar_pose = imu_pose * T_imu_lidar
        lidar_pose * exp(det_lidar) = imu_pose * exp(det_imu) * T_imu_lidar
        T_imu_lidar * exp(det_lidar) * T_imu_lidar^-1 = exp(det_imu)
        T_imu_lidar * det_lidar * T_imu_lidar^-1 = det_imu
        adj_map_imu_lidar * det_lidar = det_imu
        det_imu * det_imu^T = adj_map_imu_lidar * det_lidar * det_lidar^T * adj_map_imu_lidar^T
    */
    void _convert_odom_msg(const geometry_msgs::PoseStampedConstPtr &msg, Pose3 &imu_pose) 
    {
        imu_pose = Pose3(Rot3::Quaternion(msg->pose.orientation.w, msg->pose.orientation.x,
                                          msg->pose.orientation.y, msg->pose.orientation.z),
                                   Point3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z)) *
                         T_LIDAR_IMU;
    };

    void _calc_pose_increase(const geometry_msgs::PoseStampedConstPtr &msg, Pose3 &imu_pose_inc)
    {
        static Pose3 prev_T_map_imu = Pose3::identity();
        Pose3 cur_T_map_imu;
        _convert_odom_msg(msg, cur_T_map_imu);
        imu_pose_inc = prev_T_map_imu.transformPoseTo(cur_T_map_imu);
        prev_T_map_imu = cur_T_map_imu;
    }

    geometry_msgs::PoseStampedConstPtr _pop_next_odom_msg()
    {
        std::lock_guard<std::mutex> lock_guard(_m_odom_buf);
        geometry_msgs::PoseStampedConstPtr re(std::move(_odom_buf.front()));
        _odom_buf.pop_front();
        if(_odom_buf.size() > 1)
            ROS_INFO("drop %d odometry msg(s) for realtime performance", _odom_buf.size() - 1);
        while(_odom_buf.size() > 1)
            _odom_buf.pop_front();
        return re;
    }

    void _integrate_by_range(PreintegratedImuMeasurements& imu_acc, const ros::Time& stamp_pre, const ros::Time& stamp_cur)
    {
            auto last_stamp = stamp_pre;
            double dt;
            std::lock_guard<std::mutex> lock_guard(_m_imu_buf);
            sensor_msgs::ImuConstPtr imu_msg;
            do 
            {
                const auto& imu_msg = _imu_buf_opt.front();
                dt = (std::min(ROS_TIME(imu_msg), stamp_cur) - last_stamp).toSec();
                if(dt <= 0) break;
                last_stamp = ROS_TIME(imu_msg);
                imu_acc.integrateMeasurement(Vector3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z),
                                             Vector3(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z), dt);
                _imu_buf_opt.pop_front();
            }while(last_stamp < stamp_cur);
    }

    void _wait_for_imu()
    {
        ros::NodeHandle nh;
        ros::Rate rate(100);
        while (nh.ok())
        {
            {
                std::lock_guard<std::mutex> odom_lock_guard(_m_odom_buf);
                std::lock_guard<std::mutex> imu_lock_guard(_m_imu_buf);
                if (!_odom_buf.empty() && !_imu_buf_opt.empty())
                {
                    if (!lower(_imu_buf_opt.front(), _odom_buf.front()))
                    {
                        _odom_buf.pop_front();
                        continue;
                    }
                    if (lower(_odom_buf.front(), _imu_buf_opt.back()))
                    {
                        return;
                    }
                }
            }
            rate.sleep();
        }
    }

    void _odom_handler(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        static ros::Time last_stamp(0);
        std::lock_guard<std::mutex> lock_guard(_m_odom_buf);
        if(msg->header.stamp <= last_stamp)
            return;
        last_stamp = msg->header.stamp;
        _odom_buf.push_back(msg);
    }


    
    Vector3 PRIOR_VELOCITY;
    imuBias::ConstantBias PRIOR_BIAS;
    Vector6 BIAS_PER_SEC;
    noiseModel::Base::shared_ptr PRIOR_POSE_NOISE, PRIOR_VELOCITY_NOISE, PRIOR_BIAS_NOISE, PRIOR_LIDAR_POSE_INC_NOISE;

    Pose3 T_LIDAR_IMU;
    tf::StampedTransform P_IMU_BASE;

    double RESET_PERIOD = 8; 
    ISAM2Params ISAM_PARAMS;
    boost::shared_ptr<PreintegrationParams> IMU_ACC_PARAMS;
    gtsam::noiseModel::mEstimator::Huber::shared_ptr HUBER;
    StampedNavState _pre_nav_state;
    std::deque<sensor_msgs::ImuConstPtr> _imu_buf_odom, _imu_buf_opt;
    std::deque<geometry_msgs::PoseStampedConstPtr> _odom_buf;
    std::mutex _m_imu_buf, _m_odom_buf;
    boost::shared_ptr<PreintegratedImuMeasurements> _imu_accumulator, _imu_accumulator_opt;
    boost::shared_ptr<IncrementalFixedLagSmoother> _isam;
    std::thread _optimize_thread;
    ros::Subscriber _sub_imu, _sub_odom;
    ros::Publisher _pub_pose, _pub_twist;

    bool _is_odometer;
    bool _rotation_calibrated;
}; // class ImuOdometry

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ImuOdometry");
    ros::NodeHandle nh;
    ros::Time::waitForValid();
    ImuOdometry imu_odom;
    imu_odom.SpinOnMultiQueue();
}
