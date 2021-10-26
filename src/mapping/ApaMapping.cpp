#include <thread>
#include <queue>
#include <map>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <atomic>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf_conversions/tf_eigen.h>
#include <wloam/CloudInfo.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>

#include "utils/tfTools.hpp"
#include "utils/stampTools.hpp"
#include "utils/common_names.h"
#include "utils/my_point_type.h"
#include "factors/LidarEdgeFactor.hpp"
#include "factors/LidarPlaneFactor.hpp"
#include "factors/GPSAutoAlignFactor.hpp"

#define USE_GPSAutoAlignFactor3D

using NodeId = int32_t;
using Point3 = Eigen::Vector3d;
using Pose2 = Eigen::Isometry2d;
using Pose3 = Eigen::Isometry3d;
using Pose3Covariance = Eigen::Matrix<double, 6, 6>;
using Twist3 = Eigen::Matrix<double, 6, 1>;
using FactorParam = Eigen::Matrix<double, 3, 4>;
using PointType = wloam::RichPoint;
using CloudType = wloam::CloudType;
using CloudTypePtr = wloam::CloudTypePtr;
using CloudTypeConstPtr = wloam::CloudTypeConstPtr;
using gtsam::symbol_shorthand::G;
using gtsam::symbol_shorthand::P;
using GPSPoint = pcl::PointXYZ;

constexpr double TRAJ_RES = 0.02;

void apply_filter(pcl::VoxelGrid<PointType> &filter, const double leaf_size, const CloudTypeConstPtr &in, const CloudTypePtr &out)
{
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.setInputCloud(in);
    if(in.get() != out.get())
    {
        filter.filter(*out);
        return;
    }
    CloudTypePtr temp = boost::make_shared<CloudType>();
    filter.filter(*temp);
    out->swap(*temp);
}


void apply_filter_in_place(pcl::VoxelGrid<PointType> &filter, const double leaf_size, CloudTypePtr &in, CloudTypePtr &local_buf)
{
    local_buf->clear();
    apply_filter(filter, leaf_size, in, local_buf);
    in.swap(local_buf);
}

void normalize_pose(Pose3 &pose)
{
    Pose3 temp;
    temp.setIdentity();
    temp.rotate(Eigen::Quaterniond(pose.linear()).normalized());
    temp.pretranslate(pose.translation());
    pose = temp;
}

void send_static_transform(const geometry_msgs::TransformStamped &msg)
{
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock_gurad(mutex);
    static tf2_ros::StaticTransformBroadcaster static_br;
    static_br.sendTransform(msg);
}

struct RelativePoseStamped
{
    ros::Duration RelStamp;
    Pose3 RelPose;

    RelativePoseStamped(const ros::Duration &rel_stamp = ros::Duration(0), const Pose3 &rel_pose = Pose3::Identity())
        : RelStamp(rel_stamp), RelPose(rel_pose) {}
}; // struct RelativePoseStamped

struct MapNode
{
    using Ptr = std::shared_ptr<MapNode>;
    using ConstPtr = std::shared_ptr<const MapNode>;

    NodeId id;
    ros::Time Stamp;
    Pose3 Pose;
    Pose3Covariance PoseCov = Pose3Covariance::Identity() * 1e-12;
    std::vector<RelativePoseStamped> RelTraj;
    Twist3 Twist;
    CloudTypePtr SharpPoints, FlatPoints, FullPoints;

    MapNode() : id(0), Stamp(0), SharpPoints(boost::make_shared<CloudType>()),
                FlatPoints(boost::make_shared<CloudType>()), FullPoints(boost::make_shared<CloudType>())
    {
        Pose.setIdentity();
        RelTraj.resize(1);
        Twist.setZero();
    }

    void clear()
    {
        id = 0;
        Stamp.fromSec(0);
        Pose.setIdentity();
        RelTraj.resize(1);
        Twist.setZero();
        FlatPoints->clear();
        SharpPoints->clear();
        FullPoints->clear();
    }

    Ptr clone() const
    {
        auto copy = std::make_shared<MapNode>(*this);
        copy->SharpPoints = SharpPoints->makeShared();
        copy->FlatPoints = FlatPoints->makeShared();
        copy->FullPoints = FullPoints->makeShared();
        return copy;
    }

    bool empty() const
    {
        return SharpPoints->empty() || FlatPoints->empty() || FullPoints->empty();
    }

    size_t size() const
    {
        return std::max(std::max(SharpPoints->size(), FlatPoints->size()), FullPoints->size());
    }

    void set_data(const wloam::CloudInfoConstPtr &msg)
    {
        Stamp = msg->PointsFull.header.stamp;
        Pose.setIdentity();
        Pose.rotate(Eigen::Quaterniond(msg->Odometry.orientation.w, msg->Odometry.orientation.x,
                                       msg->Odometry.orientation.y, msg->Odometry.orientation.z));
        Pose.pretranslate(Eigen::Vector3d(msg->Odometry.position.x, msg->Odometry.position.y, msg->Odometry.position.z));
        Twist << msg->LocalTwist.angular.x, msg->LocalTwist.angular.y, msg->LocalTwist.angular.z,
            msg->LocalTwist.linear.x, msg->LocalTwist.linear.y, msg->LocalTwist.linear.z;
        pcl::fromROSMsg(msg->PointsSharp, *SharpPoints);
        pcl::fromROSMsg(msg->PointsFlat, *FlatPoints);
        pcl::fromROSMsg(msg->PointsFull, *FullPoints);
    }

    void combine(const MapNode &other, const Pose3 *node_pose = nullptr, bool ignore_path = false, bool ignore_features = false, bool ignore_map = false)
    {
        if (node_pose == nullptr)
        {
            node_pose = &other.Pose;
        }
        const auto rel_pose = Pose.inverse() * *node_pose;
        const auto rel_stamp = other.Stamp - Stamp;

        if (!ignore_path)
        {
            for (const auto &pose_stamp : other.RelTraj)
            {
                RelTraj.emplace_back(rel_stamp + pose_stamp.RelStamp, rel_pose * pose_stamp.RelPose);
            }
        }
        if (!ignore_map)
            _append_points(*FullPoints, *other.FullPoints, rel_pose);

        if (!ignore_features)
        {
            _append_points(*SharpPoints, *other.SharpPoints, rel_pose);
            _append_points(*FlatPoints, *other.FlatPoints, rel_pose);
        }
    }

    Pose3 getCenter() const
    {
        return Pose * RelTraj[RelTraj.size() / 2].RelPose;
    }

    auto getMiddleStamp() const
    {
        return Stamp + RelTraj[RelTraj.size() / 2].RelStamp;
    }

    MapNode &operator+=(const MapNode &other)
    {
        // id = std::max(id, other.id);
        // Stamp = std::max(Stamp, other.Stamp);
        combine(other);
        return *this;
    }

private:
    void _append_points(CloudType &points_out, const CloudType &extra_points, const Pose3 &pose)
    {
        if (pose.isApprox(Pose3::Identity(), 1e-8))
        {
            points_out += extra_points;
            return;
        }

        const auto start = points_out.size();
        const auto size = extra_points.size();
        points_out.resize(start + size);
        Eigen::Transform<float, 3, Eigen::Affine> transform(pose.cast<float>());
        for (int i = 0; i < size; ++i)
        {
            points_out[start + i] = pcl::transformPoint(extra_points[i], transform);
        }
    }

}; // struct MapNode

class LocalMap
{

public:
    LocalMap() : _local_map(std::make_shared<MapNode>())
    {
        setLeafSize(0.2, 0.5, 0.3);
        _key_frame_res_local = 1;
        _key_frame_history.resize(10);
    }

    void setSlidingWindow(const double length, const double res)
    {
        _key_frame_res_local = res;
        _key_frame_history.resize(int(length / res) + 1);
    }

    void setLeafSize(const double corner_leaf_size, const double surf_leaf_size, const double full_leaf_size)
    {
        _corner_leaf_size = corner_leaf_size;
        _surf_leaf_size = surf_leaf_size;
        _full_leaf_size = full_leaf_size;
    }

    bool empty() const { return _local_map->empty(); }

    bool isReady() const { return _initialized; }

    const double distance() const { return _total_accumulated_dist;}

    void initialize(const Pose3 &pose, const ros::Time &stamp)
    {
        _local_map->id = 0;
        _local_map->Pose.setIdentity();
        _local_map->Stamp = stamp;
        _last_positon = pose.translation();
        _cur_accumulated_dist = 0;
        _total_accumulated_dist = 0;
        for (auto &frame : _key_frame_history)
        {
            frame = std::make_shared<MapNode>();
        }
        _initialized = true;
    }

    void update(const MapNode::ConstPtr &new_node)
    {
        static CloudTypePtr points_ds_cur = boost::make_shared<CloudType>();
        static CloudTypePtr points_ds_history = boost::make_shared<CloudType>();

        double dist = (new_node->Pose.translation() - _last_positon).norm();
        if(dist >= TRAJ_RES)
            _last_positon = new_node->Pose.translation();
        else
            dist = 0;

        _total_accumulated_dist += dist;

        _filter.setMinimumPointsNumberPerVoxel(1);
        if (_cur_accumulated_dist < _key_frame_res_local)
            _cur_accumulated_dist += dist;
        else if (!_key_frame_history[_cur_key_frame_ind]->empty())
        {
            _cur_accumulated_dist = 0;
            _cur_key_frame_ind = (_cur_key_frame_ind + 1) % _key_frame_history.size();
            _key_frame_history[_cur_key_frame_ind]->clear();
            // _local_map->clear();
            const auto size = _key_frame_history.size();
            for (int i = 1; i < size; ++i)
            {
                _local_map->combine(*_key_frame_history[(_cur_key_frame_ind + i) % size], nullptr, true);
            }
            _filter.setMinimumPointsNumberPerVoxel(2);
        }

        _local_map->combine(*new_node, nullptr, true);

        apply_filter_in_place(_filter, _corner_leaf_size, _local_map->SharpPoints, points_ds_cur);
        apply_filter_in_place(_filter, _surf_leaf_size, _local_map->FlatPoints, points_ds_cur);
        apply_filter_in_place(_filter, _full_leaf_size, _local_map->FullPoints, points_ds_cur);

        _filter.setMinimumPointsNumberPerVoxel(1);
        _key_frame_history[_cur_key_frame_ind]->combine(*new_node, nullptr, true);

        apply_filter_in_place(_filter, _corner_leaf_size, _key_frame_history[_cur_key_frame_ind]->SharpPoints, points_ds_history);
        apply_filter_in_place(_filter, _surf_leaf_size, _key_frame_history[_cur_key_frame_ind]->FlatPoints, points_ds_history);
        apply_filter_in_place(_filter, _full_leaf_size, _key_frame_history[_cur_key_frame_ind]->FullPoints, points_ds_history);
    }

    CloudTypePtr getSharpFeatures() const { return _local_map->SharpPoints; }
    CloudTypePtr getFlatFeatures() const { return _local_map->FlatPoints; }
    CloudTypePtr getFullPoints() const { return _local_map->FullPoints; }
    const ros::Time &getStamp() const { return _local_map->Stamp; }

private:
    double _corner_leaf_size, _surf_leaf_size, _full_leaf_size;
    pcl::VoxelGrid<PointType> _filter;
    MapNode::Ptr _local_map;
    std::vector<MapNode::Ptr> _key_frame_history;
    Eigen::Vector3d _last_positon;
    double _cur_accumulated_dist, _total_accumulated_dist;
    int _cur_key_frame_ind = 0;
    bool _initialized = false;
    double _key_frame_res_local;

}; // LocalMap

class GlobalMap
{
    using KeyPoint = pcl::PointXYZ;
    using KeyCloud = pcl::PointCloud<KeyPoint>;

public:
    GlobalMap()
    {
    }

    void emplace_new(const MapNode::Ptr &new_node)
    {
        new_node->id = _key_frame_count++;
        _key_frames.push_back(new_node);
    }

    auto begin() const
    {
        return _key_frames.begin();
    }

    auto end() const
    {
        return _key_frames.end();
    }

    auto &at(size_t n) const
    {
        return _key_frames.at(n);
    }

    auto front() const
    {
        return _key_frames.front();
    }

    auto back() const
    {
        return _key_frames.back();
    }

    bool empty() const
    {
        return _key_frames.empty();
    }

    auto size() const
    {
        return _key_frames.size();
    }

    void find_neighbors(float x, float y, float z, float radius, std::vector<MapNode::ConstPtr> &neighbors) const
    {
        static pcl::KdTreeFLANN<KeyPoint> kdtree;

        neighbors.clear();
        if (_key_frames.empty())
            return;

        KeyCloud::Ptr key_frame_position(new KeyCloud);
        size_t key_frame_count = _key_frames.size();
        key_frame_position->resize(key_frame_count);

        for (size_t i = 0; i < key_frame_count; ++i)
        {
            const auto pos = _key_frames[i]->getCenter().translation();
            (*key_frame_position)[i].x = pos[0];
            (*key_frame_position)[i].y = pos[1];
            (*key_frame_position)[i].z = pos[2];
        }

        kdtree.setInputCloud(key_frame_position);

        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        KeyPoint key_p;
        key_p.x = x;
        key_p.y = y;
        key_p.z = z;
        kdtree.radiusSearch(key_p, radius, k_indices, k_sqr_distances);
        for (const auto i : k_indices)
        {
            neighbors.push_back(_key_frames.at(i));
        }
    }

private:
    std::vector<MapNode::Ptr> _key_frames;
    NodeId _key_frame_count = 0;

}; // class GlobalMap

class ApaMapping
{
#ifdef USE_GPSAutoAlignFactor3D
    using GPSAutoAlignFactor = gtsam::GPSAutoAlignFactor3D;
#else
    using GPSAutoAlignFactor = gtsam::GPSAutoAlignFactor2D;
#endif
public:
    ApaMapping()
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        ros::Time::waitForValid();

        _enable_global_optimize = private_nh.param<bool>("enable_global_optimize", false);
        _enable_loop_closure = private_nh.param<bool>("enable_loop_closure", false);
        _enable_gps_factor = private_nh.param<bool>("enable_gps_factor", false);
        _save_path = private_nh.param<bool>("save_path", false);
        _save_pcd_map = private_nh.param<bool>("save_map", false);
        _save_pcd_feature_map = private_nh.param<bool>("save_feature_map", false);
        // _use_degenerate_feature = private_nh.param<bool>("use_degenerate_feature", false);
        _use_degenerate_feature.store(false, std::memory_order_relaxed);
        _saving_path = private_nh.param<std::string>("export_path", std::getenv("HOME"));

        _key_frame_res_global = private_nh.param<double>("key_frame_res_global", 5);
        _neighbor_range_for_loop = private_nh.param<int>("neighbor_range_for_loop", 5);
        _local_map.setSlidingWindow(private_nh.param<double>("local_map_scale", 10), private_nh.param<double>("key_frame_res_local", 2));
        _loop_closure_search_radius = private_nh.param<double>("loop_closure_search_radius", 15);
        _icp_fit_score_threshold = private_nh.param<double>("icp_fit_score_threshold", 1);
        _sampling_rate_local = private_nh.param<double>("local_sampling_rate", 1);
        _sampling_rate_global = private_nh.param<double>("global_sampling_rate", 1);
        _edge_noise_threshold = private_nh.param<double>("edge_constraint_noise_threshold", 0.3);
        _plane_noise_threshold = private_nh.param<double>("plane_constraint_noise_threshold", 0.05);
        _gps_noise_xy = private_nh.param<double>("gps_noise_xy", 1);
        _gps_noise_z = private_nh.param<double>("gps_noise_z", 1);
        _min_feature_count = private_nh.param<int>("min_feature_count", 20);
        _min_noise_prior = private_nh.param<double>("min_noise_prior", 0.02);

        const auto P_lidar_gps = read_static_tf(lidar_link_frame_id, gps_link_frame_id);
        Pose3 T_lidar_gps;
        tf::poseTFToEigen(P_lidar_gps, _T_lidar_gps);

        std::vector<double> local_map_params(3), global_map_params(3), prior_between_noise(6);
        private_nh.param("local_map_resolution_corner_surf_full", local_map_params, {0.25 ,0.5 ,0.5});
        _corner_leaf_size_local = local_map_params[0];
        _surf_leaf_size_local = local_map_params[1];
        _full_leaf_size_local = local_map_params[2];
        _local_map.setLeafSize(_corner_leaf_size_local, _surf_leaf_size_local, _full_leaf_size_local);

        private_nh.param("global_map_resolution_corner_surf_full", global_map_params, {0.5, 1, 0.5});
        _corner_leaf_size_global = global_map_params[0];
        _surf_leaf_size_global = global_map_params[1];
        _full_leaf_size_global = global_map_params[2];

        private_nh.param("prior_between_noise", prior_between_noise, {0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
        _prior_between_noise.setZero();
        for(int i = 0; i < 6; ++i)
        {
            _prior_between_noise(i, i) = prior_between_noise[i] * prior_between_noise[i];
        }

        T_local_map_odom.setIdentity();
        T_map_local_map.setIdentity();
        _T_earth_map.setIdentity();
        _gps_path.header.frame_id = earth_frame_id;

        _pub_local_optimized_result(nullptr);
        _update_global_tf_tree();

        auto params = gtsam::ISAM2Params(gtsam::ISAM2GaussNewtonParams(), 0.1, 1);
        gtsam::FastMap<char, gtsam::Vector> relinearize_threshold;
        relinearize_threshold['p'] = (gtsam::Vector6() << 0.02, 0.02, 0.02, 0.1, 0.1, 0.1).finished();
        relinearize_threshold['g'] = (gtsam::Vector6() << 0.002, 0.002, 0.002, 0.01, 0.01, 0.01).finished();
        params.setRelinearizeThreshold(relinearize_threshold);
        _isam_global.reset(new gtsam::ISAM2(params));
        RobustLossFcn = gtsam::noiseModel::mEstimator::Huber::Create(0.1);

        if (_enable_gps_factor)
        {
            gtsam::Values initial_guess;
            initial_guess.insert(_gps_node_id, GPSAutoAlignFactor::AlignT(gtsam::Pose3::identity()));
            _isam_global->update(gtsam::NonlinearFactorGraph(), initial_guess);
        }

        _sub_msg = nh.subscribe<wloam::CloudInfo>(topic_cloud_info, 10, &ApaMapping::_cloud_info_msg_handler, this, ros::TransportHints().tcpNoDelay());
        _pub_odom_opt = nh.advertise<geometry_msgs::PoseStamped>(topic_odometry_lidar_optimized, 10);
        _pub_cloud_registered = nh.advertise<sensor_msgs::PointCloud2>(topic_laser_cloud_registered, 3);
        _pub_cloud_surround = nh.advertise<sensor_msgs::PointCloud2>(topic_laser_cloud_surround, 3, true);
        _pub_cloud_surround_sharp = nh.advertise<sensor_msgs::PointCloud2>(topic_laser_cloud_surround_corner, 3, true);
        _pub_cloud_surround_flat = nh.advertise<sensor_msgs::PointCloud2>(topic_laser_cloud_surround_surf, 3, true);
        _pub_cloud_map = nh.advertise<sensor_msgs::PointCloud2>(topic_laser_cloud_map, 3, true);
        _pub_selected_surf = nh.advertise<sensor_msgs::PointCloud2>("selected_surf", 3, true);
        _pub_selected_corner = nh.advertise<sensor_msgs::PointCloud2>("selected_corner", 3, true);
        _pub_selected_degenerate = nh.advertise<sensor_msgs::PointCloud2>("selected_degenerate", 3, true);
        _pub_gps_path_measured = nh.advertise<sensor_msgs::PointCloud2>("gps_path_measured", 3, true);
        _pub_key_frame_path_global = nh.advertise<nav_msgs::Path>("key_frame_path_global", 3, true);
        _pub_loop_closure_marker = nh.advertise<visualization_msgs::MarkerArray>("loop_closure", 3, true);
        _pub_optimized_path_global = nh.advertise<nav_msgs::Path>("global_path_optimized", 3, true);
        _pub_icp_registered_frame = nh.advertise<sensor_msgs::PointCloud2>("laser_cloud_registered_by_icp", 3, true);
    }

    void start_process()
    {
        _local_optimize_thread = std::thread(&ApaMapping::_local_optimize, this);
        if (_enable_global_optimize)
        {
            _global_optimize_thread = std::thread(&ApaMapping::_global_optimize, this);
            _global_map_pub_thread = std::thread(&ApaMapping::_global_map_pub_process, this);
            if (_enable_gps_factor)
            {
                ros::NodeHandle nh;
                _sub_gps_odom = nh.subscribe<nav_msgs::Odometry>(topic_gps_odom, 32, &ApaMapping::_gps_odom_msg_handler, this);
            }
        }
    }

    ~ApaMapping()
    {
        _local_optimize_thread.join();
        if (_enable_global_optimize)
        {
            _global_optimize_thread.join();
            _global_map_pub_thread.join();
        }
    }

private:
    void _local_optimize()
    {
        ros::NodeHandle nh;
        ros::Rate rate_100(100);
        pcl::VoxelGrid<PointType> filter;
        std::deque<MapNode::Ptr> recycled;
        bool delay_set_degenerate = false;
        while (nh.ok())
        {
            _mBuf.lock();
            if (_msg_buf.empty())
            {
                _mBuf.unlock();
                rate_100.sleep();
                continue;
            }
            auto msg = _msg_buf.front();
            _msg_buf.pop_front();
            _mBuf.unlock();

            MapNode::Ptr new_node;
            if(recycled.size() > 16 && recycled.front().use_count() < 2)
            {
                new_node.swap(recycled.front());
                new_node->clear();
                recycled.pop_front();
                // ROS_INFO("use recycled, size: %d", recycled.size());
            }   
            else
            {
                new_node = std::make_shared<MapNode>();
                // ROS_INFO("make new");
            }
            new_node->set_data(msg);
            const auto T_odom_lidar = new_node->Pose;
            _assosiate_transform(new_node->Pose, T_local_map_odom);

            if (!_local_map.isReady())
            {
                _local_map.initialize(new_node->Pose, new_node->Stamp);
            }

            if (!_optimize_current_pose(*new_node))
            {
                ROS_FATAL("local optimize failed, abort");
            }
            else
            {
                _update_transform(T_odom_lidar, new_node->Pose, T_local_map_odom);

                _pub_local_optimized_result(new_node);

                if (!_enable_global_optimize)
                    _pub_map_without_global_optimize(new_node);

                _local_map.update(new_node);

                if(!delay_set_degenerate && _local_map.distance() > 5)
                {
                    delay_set_degenerate = true;
                    _use_degenerate_feature.store(ros::NodeHandle("~").param<bool>("use_degenerate_feature", false), std::memory_order_relaxed); 
                }

                recycled.push_back(new_node);
                if(recycled.size() > 32)
                {
                    recycled.pop_front();
                }
                if (_enable_global_optimize)
                {
                    _send_key_frame(new_node);
                }
            }
            
            int drop_frame_count = drop_with_lock(_msg_buf, _mBuf, 2, 1);
            if (drop_frame_count > 0)
            {
                ROS_WARN("%d cloud info frame(s) be droped for real-time performance", drop_frame_count);
            }
        }
    }

    void _global_optimize()
    {
        ros::NodeHandle nh;
        ros::Rate rate(100);
        std::vector<MapNode::ConstPtr> new_nodes;
        NodeId loop_detect_indice = 1;
        NodeId gps_factor_indice = 0;
        MapNode::Ptr cur_map_node;
        gtsam::NonlinearFactorGraph graph, gps_factor_graph;
        gtsam::Values initial_guess;
        bool gps_ready = false;
        constexpr int min_gps_factor_count = 3;

        while (nh.ok())
        {
            _copy_latest_key_frame(new_nodes);

            bool graph_updated = _update_global_graph(new_nodes, cur_map_node, graph, initial_guess);
            bool loop_closed = false;
            while (_enable_loop_closure && loop_detect_indice < _global_map.size())
            {
                MapNode::ConstPtr cur_node, loop_node;
                cur_node = _global_map.at(loop_detect_indice++);
                if (_detect_loop(cur_node, loop_node) && _insert_loop_closure(loop_node, cur_node, graph, initial_guess))
                {
                    _loops.emplace_back(cur_node->id, loop_node->id);
                    loop_closed = true;
                    break;
                }
            }

            bool gps_updated = false;
            if (_enable_gps_factor && gps_factor_indice < _global_map.size())
            {
                gps_updated = _insert_gps_factor(_global_map.at(gps_factor_indice++), gps_factor_graph, initial_guess);
                if (gps_ready)
                {
                    graph.add(gps_factor_graph);
                    gps_factor_graph.resize(0);
                }
                else if (gps_factor_graph.size() >= min_gps_factor_count)
                {
                    gps_ready = true;
                }
            }

            if (graph.empty())
            {
                rate.sleep();
                continue;
            }

            _isam_global->update(graph, initial_guess);
            _isam_global->update();

            graph.resize(0);
            initial_guess.clear();

            if (loop_closed || gps_updated)
            {
                _isam_global->update();

                const auto result = _isam_global->calculateBestEstimate();
                for (const auto &node : _global_map)
                {
                    node->Pose = result.at<gtsam::Pose3>(P(node->id)).matrix();
                    normalize_pose(node->Pose);
                    node->PoseCov = _isam_global->marginalCovariance(P(node->id));
                }
                if (_enable_gps_factor)
                    _T_earth_map = result.at<GPSAutoAlignFactor::AlignT>(_gps_node_id).matrix();

                _update_transform(_last_key_pose_in_local_map, _global_map.back()->Pose, T_map_local_map);

                _update_global_tf_tree();

                _pub_loop_closure();
            }

            _update_key_frame_path_global();

            _send_latest_map_poses(loop_closed || gps_updated);
        }

        if(cur_map_node != nullptr)
        {
            pcl::VoxelGrid<CloudType::PointType> filter;
            CloudTypePtr points_ds = boost::make_shared<CloudType>();
            apply_filter_in_place(filter, _corner_leaf_size_global, cur_map_node->SharpPoints, points_ds);
            apply_filter_in_place(filter, _surf_leaf_size_global, cur_map_node->FlatPoints, points_ds);
            apply_filter_in_place(filter, _full_leaf_size_global, cur_map_node->FullPoints, points_ds);
            cur_map_node->SharpPoints->points.shrink_to_fit();
            cur_map_node->FlatPoints->points.shrink_to_fit();
            cur_map_node->FullPoints->points.shrink_to_fit();
            _assosiate_transform(cur_map_node->Pose, T_map_local_map);
            _global_map.emplace_new(cur_map_node);
        }

        _save_global_optimized_result();
    }

    void _save_global_optimized_result() const
    {
        constexpr char path_filename[] = "global_path.tum";
        constexpr char transform_file_name[] = "T_earth_map.json";
        constexpr char map_filename[] = "global_map.pcd";
        constexpr char feature_corner_filename[] = "global_corner_feature_map.pcd";
        constexpr char feature_surf_filename[] = "global_surf_feature_map.pcd";

        pcl::PCDWriter writer;
        pcl::VoxelGrid<PointType> filter;

        if (_global_map.empty() || !(_save_path || _save_pcd_map || _save_pcd_feature_map))
        {
            return;
        }

        MapNode map;
        for (const auto &node : _global_map)
        {
            map.combine(*node, nullptr, !_save_path, !_save_pcd_feature_map, !_save_pcd_map);
        }

        if (_save_path)
        {
            std::cout << "Please wait for saving path file " << path_filename << "\n";
            std::ofstream out(_saving_path + "/" + path_filename, std::ios_base::out | std::ios_base::trunc);
            if (!out.is_open())
                std::cout << "Open " << _saving_path << " failed, abort\n";
            else if (map.RelTraj.size() < 2)
                std::cout << "Not enough pose for saving, abort\n";
            else
            {
                auto p = map.RelTraj.begin() + 1;
                out << scientific << setprecision(18);
                const auto T_earth_cur = _T_earth_map * map.Pose;
                while (p != map.RelTraj.end())
                {
                    Pose3 pose = T_earth_cur * p->RelPose;
                    const Eigen::Vector3d &t = pose.translation();
                    const Eigen::Quaterniond q(pose.linear());
                    out << (map.Stamp + p->RelStamp).toSec()
                        << " " << t.x() << " " << t.y() << " " << t.z()
                        << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
                    ++p;
                }
                std::cout << "saved " << map.RelTraj.size() - 1 << " pose to " << _saving_path + "/" + path_filename << std::endl;
            }
            out.close();
        }

        if (_save_path && _enable_gps_factor)
        {
            std::cout << "Please wait for saving path file " << transform_file_name << "\n";
            std::ofstream out(_saving_path + "/" + transform_file_name, std::ios_base::out | std::ios_base::trunc);
            if (!out.is_open())
                std::cout << "Open " << _saving_path << " failed, abort\n";
            else
            {
                Eigen::Quaterniond q(_T_earth_map.linear());
                out << scientific << setprecision(18) 
                    << "{\n" 
                    << "\t\"x\": " << _T_earth_map.translation().x() << ",\n"
                    << "\t\"y\": " << _T_earth_map.translation().y() << ",\n" 
                    << "\t\"z\": " << _T_earth_map.translation().z() << ",\n" 
                    << "\t\"qx\": " << q.x() << ",\n" 
                    << "\t\"qy\": " << q.y() << ",\n" 
                    << "\t\"qz\": " << q.z() << ",\n" 
                    << "\t\"qw\": " << q.w() << "\n}";

                std::cout << "saved T_" << earth_frame_id << "_" << map_frame_id << " to " << _saving_path + "/" + transform_file_name << std::endl;
            }
            out.close();
        }

        if (_save_pcd_map)
        {
            std::cout << "Please wait for saving pcd file " << map_filename << "\n";

            apply_filter(filter, _full_leaf_size_global, map.FullPoints, map.FullPoints);

            pcl::transformPointCloud(*map.FullPoints, *map.FullPoints, _T_earth_map.matrix());

            writer.writeASCII(_saving_path + "/" + map_filename, *map.FullPoints);

            std::cout << "merged " << _global_map.size() << " local map to global map " << _saving_path + "/" + map_filename << std::endl;
        }

        if (_save_pcd_feature_map)
        {
            std::cout << "Please wait for saving pcd file " << feature_corner_filename << "\n";

            apply_filter(filter, _corner_leaf_size_global, map.SharpPoints, map.SharpPoints);

            pcl::transformPointCloud(*map.SharpPoints, *map.SharpPoints, _T_earth_map.matrix());

            writer.writeASCII(_saving_path + "/" + feature_corner_filename, *map.SharpPoints);

            std::cout << "merged " << _global_map.size() << " local corner feature map to global corner map " << _saving_path + "/" + feature_corner_filename << std::endl;

            std::cout << "Please wait for saving pcd file " << feature_surf_filename << "\n";

            apply_filter(filter, _surf_leaf_size_global, map.FlatPoints, map.FlatPoints);

            pcl::transformPointCloud(*map.FlatPoints, *map.FlatPoints, _T_earth_map.matrix());

            writer.writeASCII(_saving_path + "/" + feature_surf_filename, *map.FlatPoints);

            std::cout << "merged " << _global_map.size() << " local surf feature map to global surf map " << _saving_path + "/" + feature_surf_filename << std::endl;
        }
    }

    void _global_map_pub_process()
    {
        ros::NodeHandle nh;
        ros::Rate rate(10);
        ros::Time last_pub_time(0);
        sensor_msgs::PointCloud2 cloud_msg;
        nav_msgs::Path path;
        path.header.frame_id = map_frame_id;
        MapNode map;
        std::vector<Pose3> map_node_poses;
        NodeId node_ind = 0;
        bool refresh_flag = false;
        unsigned int skip_count = 0;

        pcl::VoxelGrid<PointType> filter;

        while (nh.ok())
        {
            rate.sleep();
            bool need_cloud = _pub_cloud_map.getNumSubscribers() > 0;
            bool need_path = _pub_optimized_path_global.getNumSubscribers() > 0; 
            if (!need_cloud && !need_path)
                continue;
            const ros::Time now = ros::Time::now();
            if ((now - last_pub_time).toSec() < 3)
                continue;
            last_pub_time = now;

            _copy_latest_map_poses(map_node_poses, refresh_flag);

            if (refresh_flag)
            {
                map.clear();
                node_ind = 0;
                ROS_INFO("global map refreshed!!");
            }

            bool map_update = false;
            while (node_ind < map_node_poses.size())
            {
                map.combine(*_global_map.at(node_ind), &map_node_poses[node_ind], false, true, false);
                ++node_ind;
                // if(node_ind % 8 == 0)
                //     apply_filter(filter, _full_leaf_size_global, map.FullPoints, map.FullPoints);
                map_update = true;
            }

            if (map_update)
            {
                // if(node_ind % 8 != 0)
                if (need_cloud)
                {
                    apply_filter(filter, _full_leaf_size_global, map.FullPoints, map.FullPoints);
                    pcl::toROSMsg(*map.FullPoints, cloud_msg);
                    cloud_msg.header.frame_id = map_frame_id;
                    cloud_msg.header.stamp = now;
                    _pub_cloud_map.publish(cloud_msg);
                }

                if (need_path && map.RelTraj.size() > 2)
                {
                    path.poses.clear();
                    path.header.stamp = now;
                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id = map_frame_id;

                    auto p = map.RelTraj.begin() + 1;
                    pose.header.stamp = map.Stamp + p->RelStamp;
                    tf::poseEigenToMsg(map.Pose * p->RelPose, pose.pose);
                    path.poses.push_back(pose);
                    ++p;

                    while (p != map.RelTraj.end())
                    {
                        pose.header.stamp = map.Stamp + p->RelStamp;
                        tf::poseEigenToMsg(map.Pose * p->RelPose, pose.pose);
                        path.poses.push_back(pose);
                        ++p;
                    }
                    _pub_optimized_path_global.publish(path);
                }
            }
        }
    }

    bool _detect_loop(const MapNode::ConstPtr &cur_node, MapNode::ConstPtr &loop_node) const
    {
        const Eigen::Vector3d cur_center = cur_node->getCenter().translation();
        const auto cur_id = cur_node->id;
        std::vector<MapNode::ConstPtr> neighbors;
        _global_map.find_neighbors(cur_center[0], cur_center[1], cur_center[2], _loop_closure_search_radius, neighbors);
        bool flag = false;
        for (const auto &node : neighbors)
        {
            if (cur_id > node->id && (cur_id - node->id) * _key_frame_res_global > _loop_closure_search_radius * 1.5)
            {
                loop_node = node;
                flag = true;
                break;
            }
        }
        return flag;
    }

    bool _update_global_graph(std::vector<MapNode::ConstPtr> &new_nodes, MapNode::Ptr& cur_map_node, gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial_guess)
    {
        static MapNode::Ptr last_map_node;
        static pcl::VoxelGrid<CloudType::PointType> filter;
        static CloudTypePtr points_ds = boost::make_shared<CloudType>();
        static Pose3 last_node_pose;
        static ros::Time last_node_stamp;
        static double accumulate_dist;
        bool graph_updated = false;

        if (new_nodes.empty())
            return false;

        if (cur_map_node == nullptr)
        {
            cur_map_node = new_nodes[0]->clone();
            accumulate_dist = 0;
            last_node_pose = new_nodes[0]->Pose;
            last_node_stamp = new_nodes[0]->Stamp;
            new_nodes.erase(new_nodes.begin());
        }

        for (auto &node : new_nodes)
        {     
            if (accumulate_dist < _key_frame_res_global)
            {
                const auto rel_pose = last_node_pose.inverse() * node->Pose;
                const auto dist = rel_pose.translation().norm();
                if (dist >= TRAJ_RES )
                {
                    last_node_pose = node->Pose;
                    accumulate_dist += dist;
                }

                if (dist >= TRAJ_RES || Eigen::AngleAxisd(rel_pose.linear()).angle() > M_PI / 180.0)
                    cur_map_node->combine(*node, nullptr, dist < TRAJ_RES);

                if((node->Stamp - last_node_stamp) > ros::Duration(10,0))
                {
                    last_node_stamp = node->Stamp;
                    ROS_INFO("current node accumulating data over 10 seconds, apply filter");
                    apply_filter_in_place(filter, _corner_leaf_size_global, cur_map_node->SharpPoints, points_ds);
                    apply_filter_in_place(filter, _surf_leaf_size_global, cur_map_node->FlatPoints, points_ds);
                    apply_filter_in_place(filter, _full_leaf_size_global, cur_map_node->FullPoints, points_ds);
                }
            }
            else
            {
                apply_filter_in_place(filter, _corner_leaf_size_global, cur_map_node->SharpPoints, points_ds);
                apply_filter_in_place(filter, _surf_leaf_size_global, cur_map_node->FlatPoints, points_ds);
                apply_filter_in_place(filter, _full_leaf_size_global, cur_map_node->FullPoints, points_ds);
                cur_map_node->SharpPoints->points.shrink_to_fit();
                cur_map_node->FlatPoints->points.shrink_to_fit();
                cur_map_node->FullPoints->points.shrink_to_fit();

                _last_key_pose_in_local_map = cur_map_node->Pose;
                _assosiate_transform(cur_map_node->Pose, T_map_local_map);
                _global_map.emplace_new(cur_map_node);
                initial_guess.insert(P(cur_map_node->id), gtsam::Pose3(cur_map_node->Pose.matrix()));
                if (last_map_node != nullptr)
                {
                    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(P(last_map_node->id), P(cur_map_node->id), gtsam::Pose3((last_map_node->Pose.inverse() * cur_map_node->Pose).matrix()), gtsam::noiseModel::Gaussian::Covariance(cur_map_node->PoseCov + _prior_between_noise));
                    ROS_INFO("add node %d and node %d as neighbor", last_map_node->id, cur_map_node->id);
                }
                else
                {
                    graph.addPrior(P(cur_map_node->id), gtsam::Pose3(cur_map_node->Pose.matrix()), gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector6() << 0.001, 0.001, 0.001, 0.01, 0.01, 0.01).finished()));
                    ROS_INFO("add node %d as prior", cur_map_node->id);
                }

                accumulate_dist = 0;
                last_node_pose = node->Pose;
                last_node_stamp = node->Stamp;
                last_map_node.swap(cur_map_node);
                cur_map_node = node->clone();
                graph_updated = true;
            }
        }
        new_nodes.clear();
        return graph_updated;
    }

    bool _insert_gps_factor(const MapNode::Ptr &node, gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial_guess)
    {
        double x, y, z;
        if (!_lookup_gps_position(node->Stamp, x, y, z))
            return false;
#ifdef USE_GPSAutoAlignFactor3D
        graph.emplace_shared<GPSAutoAlignFactor>(_gps_node_id, P(node->id), gtsam::Point3(x, y, z), _T_lidar_gps, gtsam::noiseModel::Robust::Create(RobustLossFcn, gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector3() << _gps_noise_xy, _gps_noise_xy, _gps_noise_z).finished())));
#else
        graph.emplace_shared<GPSAutoAlignFactor>(_gps_node_id, node->id, gtsam::Point2(x, y), _T_lidar_gps, gtsam::noiseModel::Robust::Create(RobustLossFcn, gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector2() << _gps_noise_xy, _gps_noise_xy).finished())));
#endif

        ROS_INFO("add GPSFactor to node: %d", node->id);
        return true;
    }

    bool _lookup_gps_position(const ros::Time &stamp, double &x, double &y, double &z)
    {
        static nav_msgs::OdometryConstPtr msg_pre(nullptr);
        std::lock_guard<std::mutex> lock_guard(_m_gps_msg_buf);

        if(_gps_msg_buf.empty() || stamp < ROS_TIME(_gps_msg_buf.front())) return false;
        auto next = std::upper_bound(_gps_msg_buf.begin(), _gps_msg_buf.end(), stamp, 
        [](const ros::Time& stamp, const nav_msgs::OdometryConstPtr& mea) {return stamp < ROS_TIME(mea);});

        if(next == _gps_msg_buf.end()) return false;

        auto pre = next - 1;

        double time_diff_pre = (stamp - ROS_TIME(*pre)).toSec();
        double time_diff_next = (ROS_TIME(*next) - stamp).toSec();
        bool valid = time_diff_pre < 1 && time_diff_next < 1;

        if (valid)
        {
            double s = time_diff_pre / (time_diff_pre + time_diff_next);
            x = (*pre)->pose.pose.position.x + s * ((*next)->pose.pose.position.x - (*pre)->pose.pose.position.x);
            y = (*pre)->pose.pose.position.y + s * ((*next)->pose.pose.position.y - (*pre)->pose.pose.position.y);
            z = (*pre)->pose.pose.position.z + s * ((*next)->pose.pose.position.z - (*pre)->pose.pose.position.z);
        }

        _gps_msg_buf.erase(_gps_msg_buf.begin(), pre);

        return valid;
    }

    bool _insert_loop_closure(const MapNode::ConstPtr &node1, const MapNode::ConstPtr &node2, gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial_guess) const
    {
        auto rel_pose = node1->Pose.inverse() * node2->Pose;
        normalize_pose(rel_pose);
        if (!_icp_optimize(node1, node2, rel_pose))
        {
            ROS_WARN("icp converge fail, abort closure");
            return false;
        };

        if (!_insert_neighbor_node_to_graph(graph, node1, node2, rel_pose))
        {
            ROS_WARN("loop closure: failed when create graph, abort");
            return false;
        }
        ROS_INFO("loop closure: between %d and %d", node1->id, node2->id);

        return true;
    }

    bool _insert_neighbor_node_to_graph(gtsam::NonlinearFactorGraph &graph, const MapNode::ConstPtr &node1, const MapNode::ConstPtr &node2, const Pose3 &rel_pose) const
    {
        static pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>);
        static pcl::VoxelGrid<PointType> filter;
        static CloudTypePtr feature_points = boost::make_shared<CloudType>();
        static gtsam::NonlinearFactorGraph temp_graph;
        static gtsam::Values initial_guess;
        std::vector<FactorParam> edge_factors, plane_factors, degenerate_factors;
        int edge_factor_count, plane_factor_count, degenerate_factor_count;

        temp_graph.resize(0);
        initial_guess.clear();
        edge_factor_count = plane_factor_count = degenerate_factor_count = 0;

        auto add_factors = [&](const MapNode::ConstPtr &u, const MapNode::ConstPtr &v, const Pose3 &rel_pose) {
            edge_factors.clear();
            plane_factors.clear();
            degenerate_factors.clear();

            feature_points->clear();
            apply_filter(filter, _corner_leaf_size_global * _sampling_rate_global, v->SharpPoints, feature_points);
            _search_near_edges(kdtree, u->SharpPoints, feature_points, rel_pose, edge_factors, degenerate_factors);

            feature_points->clear();
            apply_filter(filter, _surf_leaf_size_global * _sampling_rate_global, v->FlatPoints, feature_points);
            _search_near_planes(kdtree, u->FlatPoints, feature_points, rel_pose, plane_factors);

            for (const auto &param : edge_factors)
                temp_graph.emplace_shared<gtsam::LidarEdgeFactor2>(u->id, v->id, param.col(0), param.col(1), param.col(2), gtsam::noiseModel::Robust::Create(RobustLossFcn, gtsam::noiseModel::Isotropic::Sigma(3, param.col(3)[0])));
            // temp_graph.emplace_shared<gtsam::LidarEdgeProjectedFactor2>(u->id, v->id, param.col(0), param.col(1), param.col(2), gtsam::noiseModel::Robust::Create(RobustLossFcn, gtsam::noiseModel::Isotropic::Sigma(3, param.col(3)[0])));
            for (const auto &param : plane_factors)
                temp_graph.emplace_shared<gtsam::LidarPlaneFactor2>(u->id, v->id, param.col(0), param.col(1), param.col(2), gtsam::noiseModel::Robust::Create(RobustLossFcn, gtsam::noiseModel::Isotropic::Sigma(1, param.col(3)[0])));
            for (const auto &param : degenerate_factors)
                temp_graph.emplace_shared<gtsam::LidarPlaneFactor2>(u->id, v->id, param.col(0), param.col(1), param.col(2), gtsam::noiseModel::Robust::Create(RobustLossFcn, gtsam::noiseModel::Isotropic::Sigma(1, param.col(3)[0])));

            edge_factor_count += edge_factors.size();
            plane_factor_count += plane_factors.size();
            degenerate_factor_count += degenerate_factors.size();
        };

        add_factors(node1, node2, rel_pose);
        add_factors(node2, node1, rel_pose.inverse());

        if (edge_factor_count + plane_factor_count + degenerate_factor_count < _min_feature_count)
        {
            ROS_ERROR("edge, plane and degenerate less than %d, fail to create graph", _min_feature_count);
            return false;
        }

        ROS_INFO("create graph with %d edge_factors, %d plane_factors and %d degenerate_factors between node %d and %d", edge_factor_count, plane_factor_count, degenerate_factor_count, node1->id, node2->id);

        temp_graph.addPrior(node1->id, gtsam::Pose3::identity(), gtsam::noiseModel::Constrained::Diagonal::Sigmas(gtsam::Vector6::Zero()));
        initial_guess.insert(node1->id, gtsam::Pose3::identity());
        initial_guess.insert(node2->id, gtsam::Pose3(rel_pose.matrix()));

        gtsam::LevenbergMarquardtParams op_params;
        gtsam::LevenbergMarquardtParams::SetCeresDefaults(&op_params);
        op_params.setLinearSolverType("SEQUENTIAL_QR");

        const auto result = gtsam::LevenbergMarquardtOptimizer(temp_graph, initial_guess, op_params).optimize();
        const auto rel_pose_optimized = result.at(node2->id).cast<gtsam::Pose3>();
        gtsam::Marginals marginal(temp_graph, result, gtsam::Marginals::QR);
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(P(node1->id), P(node2->id), rel_pose_optimized, gtsam::noiseModel::Gaussian::Covariance(marginal.marginalCovariance(node2->id)));
        return true;
    }

    bool _optimize_current_pose(MapNode &node)
    {
        static gtsam::NonlinearFactorGraph graph;
        static gtsam::Values initial_guess;
        static NodeId pose_ind = 0;
        static pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>);
        static pcl::VoxelGrid<PointType> filter;

        if (_local_map.empty())
            return true;

        // static uint64_t n = 0;
        // static double sum = 0;
        // ros::Time t1 = ros::Time::now();

        static std::vector<FactorParam> edge_factors, plane_factors, degenerate_factors;
        static CloudTypePtr feature_points = boost::make_shared<CloudType>();
        edge_factors.clear();
        plane_factors.clear();
        degenerate_factors.clear();

        feature_points->clear();
        apply_filter(filter, _corner_leaf_size_local * _sampling_rate_local, node.SharpPoints, feature_points);
        _search_near_edges(kdtree, _local_map.getSharpFeatures(), feature_points, node.Pose, edge_factors, degenerate_factors);

        feature_points->clear();
        apply_filter(filter, _surf_leaf_size_local * _sampling_rate_local, node.FlatPoints, feature_points);
        _search_near_planes(kdtree, _local_map.getFlatFeatures(), feature_points, node.Pose, plane_factors);

        if (plane_factors.size() + edge_factors.size() < _min_feature_count)
            return false;

        graph.resize(0);
        initial_guess.clear();
        // double avg_edge_sigma, avg_plain_sigma, avg_degerate_sigma;
        // avg_edge_sigma = avg_plain_sigma = avg_degerate_sigma = 0;
        for (const auto &param : edge_factors)
        {
            // graph.emplace_shared<gtsam::LidarEdgeProjectedFactor1>(pose_ind, param.col(0), param.col(1), param.col(2), edge_noise);
            graph.emplace_shared<gtsam::LidarEdgeFactor1>(pose_ind, param.col(0), param.col(1), param.col(2), gtsam::noiseModel::Robust::Create(RobustLossFcn, gtsam::noiseModel::Isotropic::Sigma(3, param.col(3)[0])));
            // avg_edge_sigma += param.col(3)[0];
        }
        for (const auto &param : plane_factors)
        {
            graph.emplace_shared<gtsam::LidarPlaneFactor1>(pose_ind, param.col(0), param.col(1), param.col(2), gtsam::noiseModel::Robust::Create(RobustLossFcn, gtsam::noiseModel::Isotropic::Sigma(1, param.col(3)[0])));
            // avg_plain_sigma += param.col(3)[0];
        }
        for (const auto &param : degenerate_factors)
        {
            graph.emplace_shared<gtsam::LidarPlaneFactor1>(pose_ind, param.col(0), param.col(1), param.col(2), gtsam::noiseModel::Robust::Create(RobustLossFcn, gtsam::noiseModel::Isotropic::Sigma(1, param.col(3)[0])));
            // avg_degerate_sigma += param.col(3)[0];
        }

        // std::cout << "avg_edge_sigma:" << avg_edge_sigma / (edge_factors.size()+1);
        // std::cout << "\navg_plain_sigma:" << avg_plain_sigma / (plane_factors.size()+1);
        // std::cout << "\navg_degerate_sigma:" << avg_degerate_sigma / (degenerate_factors.size()+1) << std::endl;

        // graph.addPrior(pose_ind, gtsam::Pose3(node.Pose.matrix()), gtsam::noiseModel::Robust::Create(RobustLossFcn, gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector6() << M_PI / 180 * 5, M_PI / 180 * 5, M_PI / 180 * 5, 0.1, 0.1, 0.1).finished())));

        // _apply_rand_pose_noise(node.Pose);
        initial_guess.insert(pose_ind, gtsam::Pose3(node.Pose.matrix()));

        static gtsam::LevenbergMarquardtParams op_params = gtsam::LevenbergMarquardtParams::CeresDefaults();
        // op_params.setVerbosityLM("SUMMARY");
        // gtsam::DoglegParams op_params;
        // op_params.setMaxIterations(30);
        // op_params.setRelativeErrorTol(1e-8);
        // op_params.setAbsoluteErrorTol(0);
        // op_params.setVerbosityDL("VERBOSE");
        // op_params.setVerbosity("DELTA");
        op_params.setLinearSolverType("SEQUENTIAL_QR");

        //average 15 ms
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_guess, op_params);
        const auto &result = optimizer.optimize();
        node.Pose = result.at(pose_ind).cast<gtsam::Pose3>().matrix();

        //average 9.5 ms
        // node.Pose = gtsam::DoglegOptimizer(graph, initial_guess, op_params).optimize().at(pose_ind).cast<gtsam::Pose3>().matrix();

        // ros::Time t2 = ros::Time::now();
        // sum += (t2-t1).toSec();
        // ++n;
        // std::cout << "average local_optimize_time: " << sum / n << "\n";

        gtsam::Marginals marginal(graph, result, gtsam::Marginals::QR);
        node.PoseCov = marginal.marginalCovariance(pose_ind);

        _visualize_factors(edge_factors, node.Stamp, node.Pose, 0, 100, _pub_selected_corner);
        _visualize_factors(plane_factors, node.Stamp, node.Pose, 150, 250, _pub_selected_surf);
        _visualize_factors(degenerate_factors, node.Stamp, node.Pose, 225, 125, _pub_selected_degenerate);
        return true;
    }

    bool _icp_optimize(const MapNode::ConstPtr &node_pre, const MapNode::ConstPtr &node_cur, Pose3 &rel_pose) const
    {
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        static pcl::VoxelGrid<PointType> filter;
        static MapNode target_to_align;
        static CloudType aligned_cloud;

        double max_correspond_dist = _loop_closure_search_radius;
        icp.setMaxCorrespondenceDistance(max_correspond_dist);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        target_to_align.clear();
        target_to_align.Pose = node_pre->Pose;
        pcl::copyPointCloud(*node_pre->FullPoints, *target_to_align.FullPoints);

        const auto pre_id = node_pre->id;
        for (int i = 1; i <= _neighbor_range_for_loop; ++i)
        {
            if (pre_id + i >= _global_map.size() || pre_id + i == node_cur->id)
                break;
            target_to_align.combine(*_global_map.at(pre_id + i), nullptr, true, true);
        }
        for (int i = 1; i <= _neighbor_range_for_loop; ++i)
        {
            if (pre_id - i <= 0 || pre_id - i == node_cur->id)
                break;
            target_to_align.combine(*_global_map.at(pre_id - i), nullptr, true, true);
        }

        apply_filter(filter, _full_leaf_size_global, target_to_align.FullPoints, target_to_align.FullPoints);

        *target_to_align.SharpPoints += *node_cur->SharpPoints;
        *target_to_align.SharpPoints += *node_cur->FlatPoints;
        apply_filter(filter, _full_leaf_size_global, target_to_align.SharpPoints, target_to_align.SharpPoints);

        icp.setInputSource(target_to_align.SharpPoints);
        icp.setInputTarget(target_to_align.FullPoints);

        aligned_cloud.clear();
        icp.align(aligned_cloud, rel_pose.matrix().cast<float>());
        rel_pose = icp.getFinalTransformation().cast<double>();
        double fit_score = icp.getFitnessScore(max_correspond_dist);
        ROS_INFO("icp fitness score: %f", fit_score);

        if (_pub_icp_registered_frame.getNumSubscribers() > 0)
        {
            for (auto &p : aligned_cloud)
            {
                p.intensity = 128;
            }
            aligned_cloud += *target_to_align.FullPoints;
            pcl::transformPointCloud(aligned_cloud, aligned_cloud, node_pre->Pose.matrix());
            sensor_msgs::PointCloud2Ptr aligned_msg = boost::make_shared<sensor_msgs::PointCloud2>();
            pcl::toROSMsg(aligned_cloud, *aligned_msg);
            aligned_msg->header.frame_id = map_frame_id;
            aligned_msg->header.stamp = ros::Time::now();
            _pub_icp_registered_frame.publish(aligned_msg);
        }
        return icp.hasConverged() && fit_score < _icp_fit_score_threshold;
    }

    void _search_near_edges(const pcl::KdTreeFLANN<PointType>::Ptr &kdtree, const CloudTypeConstPtr &map_points, const CloudTypeConstPtr &feature_points, const Pose3 init_pose, std::vector<FactorParam> &params, std::vector<FactorParam> &params_degenerate) const
    {
        constexpr int K = 10;
        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        Eigen::Transform<float, 3, Eigen::Affine> transform(init_pose.cast<float>());
        params.clear();
        params_degenerate.clear();
        kdtree->setInputCloud(map_points);

        for (const auto &point : *feature_points)
        {
            const auto point_tranformed = pcl::transformPoint(point, transform);
            kdtree->nearestKSearch(point_tranformed, K, k_indices, k_sqr_distances);
            if (k_sqr_distances[K - 1] > 1.0)
                continue;
            Eigen::Matrix<double, K, 3> near_points;
            for (int i = 0; i < K; ++i)
            {
                const auto &p = map_points->at(k_indices[i]);
                near_points.row(i) << p.x, p.y, p.z;
            }
            Eigen::Vector3d center = near_points.colwise().sum() / K;
            near_points.rowwise() -= center.transpose();
            Eigen::Matrix3d covMat = near_points.transpose() * near_points;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
            Eigen::Vector3d eigen_val = saes.eigenvalues().cwiseSqrt() / std::sqrt(K - 1);

            const auto sigma = std::sqrt(eigen_val[0] * eigen_val[0] + eigen_val[1] * eigen_val[1]);

            if (_use_degenerate_feature.load(std::memory_order_relaxed) && eigen_val[0] < _plane_noise_threshold && eigen_val[1] * 3 > _corner_leaf_size_local && eigen_val[2] * 3 > _corner_leaf_size_local)
            {
                FactorParam param;
                param.col(0) = center;
                param.col(1) = saes.eigenvectors().col(0).normalized();
                param.col(2) << point.x, point.y, point.z;
                param.col(3)[0] = std::max(std::sqrt(eigen_val[0] * eigen_val[0] + _plane_noise_threshold * _plane_noise_threshold), _min_noise_prior);
                // param.col(3)[0] = std::max(eigen_val[0] + _plane_noise_threshold, _min_noise_prior);
                params_degenerate.push_back(param);
            }
            else if (sigma < _edge_noise_threshold)
            {
                FactorParam param;
                param.col(0) = center;
                param.col(1) = saes.eigenvectors().col(2).normalized();
                param.col(2) << point.x, point.y, point.z;
                param.col(3)[0] = std::max(sigma, _min_noise_prior);
                params.push_back(param);
            }
        }
    }

    void _search_near_planes(const pcl::KdTreeFLANN<PointType>::Ptr &kdtree, const CloudTypeConstPtr &map_points, const CloudTypeConstPtr &feature_points, const Pose3 init_pose, std::vector<FactorParam> &params) const
    {
        constexpr int K = 5;
        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        Eigen::Transform<float, 3, Eigen::Affine> transform(init_pose.cast<float>());
        params.clear();
        kdtree->setInputCloud(map_points);

        for (const auto &point : *feature_points)
        {
            const auto point_tranformed = pcl::transformPoint(point, transform);
            kdtree->nearestKSearch(point_tranformed, K, k_indices, k_sqr_distances);
            if (k_sqr_distances[K - 1] > 5.0)
                continue;
            Eigen::Matrix<double, K, 3> near_points;
            for (int i = 0; i < K; ++i)
            {
                const auto &p = map_points->at(k_indices[i]);
                near_points.row(i) << p.x, p.y, p.z;
            }
            Eigen::Vector3d center = near_points.colwise().sum() / K;
            near_points.rowwise() -= center.transpose();
            Eigen::Matrix3d covMat = near_points.transpose() * near_points;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
            Eigen::Vector3d eigen_val = saes.eigenvalues().cwiseSqrt() / std::sqrt(K - 1);

            if (eigen_val[0] < _plane_noise_threshold && eigen_val[1] * 3 > _surf_leaf_size_local && eigen_val[2] * 3 > _surf_leaf_size_local)
            {
                FactorParam param;
                param.col(0) = center;
                param.col(1) = saes.eigenvectors().col(0).normalized();
                param.col(2) << point.x, point.y, point.z;
                param.col(3)[0] = std::max(eigen_val[0], _min_noise_prior);
                params.push_back(param);
            }
        }
    }

    void _convert_factor_params_to_cloud(const std::vector<FactorParam> &params, CloudType &cloud, const Pose3 &pose_map, const Pose3 &pose_feature, double intensity_map, double intensity_feature) const
    {
        cloud.clear();
        Eigen::Transform<float, 3, Eigen::Affine> transform_map(pose_map.cast<float>());
        Eigen::Transform<float, 3, Eigen::Affine> transform_feature(pose_feature.cast<float>());
        PointType p;
        Eigen::Vector3d p1, p2;
        p.intensity = intensity_map;
        for (const auto &param : params)
        {
            p.x = param.col(0).x();
            p.y = param.col(0).y();
            p.z = param.col(0).z();
            cloud.push_back(p);
            for(int i = 1; i <= 5; ++i)
            {
                p1 = param.col(0) + param.col(1) * 0.1 * i;
                p2 = param.col(0) - param.col(1) * 0.1 * i;
                p.x = p1.x();
                p.y = p1.y();
                p.z = p1.z();
                cloud.push_back(p);
                p.x = p2.x();
                p.y = p2.y();
                p.z = p2.z();
                cloud.push_back(p);
            }
            // p.x = param.col(2).x();
            // p.y = param.col(2).y();
            // p.z = param.col(2).z();
            // p.intensity = intensity_feature;
            // cloud.push_back(pcl::transformPoint(p, transform_feature));
        }
    }

    void _visualize_factors(const std::vector<FactorParam> &params, const ros::Time &stamp, const Pose3 &pose_feature, int color_feature, int color_point, const ros::Publisher &pub) const
    {
        if(params.empty()) return;
        static CloudType cloud;
        static sensor_msgs::PointCloud2 msg;
        static Pose3 pose_map;
        cloud.clear();
        if (pub.getNumSubscribers() > 0)
        {
            pose_map.setIdentity();
            _convert_factor_params_to_cloud(params, cloud, pose_map, pose_feature, color_feature, color_point);
            pcl::toROSMsg(cloud, msg);
            msg.header.frame_id = local_map_frame_id;
            msg.header.stamp = stamp;
            pub.publish(msg);
        }
    }

    void _update_gps_path_measured(const double x, const double y, const double z, const ros::Time& stamp)
    {
        if (!_gps_path.empty())
        {
            const double x_diff = _gps_path.back().x - x;
            const double y_diff = _gps_path.back().y - y;
            const double z_diff = _gps_path.back().z - z;
            if (x_diff * x_diff + y_diff * y_diff + z_diff * z_diff < TRAJ_RES * TRAJ_RES)
                return;
        }

        GPSPoint p;
        p.x = x;
        p.y = y;
        p.z = z;
        _gps_path.push_back(p);

        if (_pub_gps_path_measured.getNumSubscribers() > 0)
        {
            static sensor_msgs::PointCloud2 path_msg;
            pcl::toROSMsg(_gps_path, path_msg);
            _pub_gps_path_measured.publish(path_msg);
        }
    }

    void _pub_local_optimized_result(const MapNode::ConstPtr &node)
    {
        static geometry_msgs::PoseStamped odom_opt;
        static geometry_msgs::TransformStamped tf_msg;
        static tf::Transform T;
        static sensor_msgs::PointCloud2 cloud_msg;
        static ros::Time last_time(0);

        if (node != nullptr)
        {
            odom_opt.header.frame_id = local_map_frame_id;
            odom_opt.header.stamp = node->Stamp;
            tf::poseEigenToMsg(node->Pose, odom_opt.pose);
            _pub_odom_opt.publish(odom_opt);
        }
        tf_msg.header.frame_id = local_map_frame_id;
        tf_msg.child_frame_id = odom_frame_id;

        tf::poseEigenToTF(T_local_map_odom, T);
        tf::transformTFToMsg(T, tf_msg.transform);
        send_static_transform(tf_msg);

        ros::Time last_registered_cloud_stamp(0);
        if (node != nullptr && (node->Stamp - last_registered_cloud_stamp).toSec() > 0.15 && _pub_cloud_registered.getNumSubscribers() > 0)
        {
            static pcl::VoxelGrid<CloudType::PointType> filter;
            static CloudType cloud;
            last_registered_cloud_stamp = node->Stamp;
            filter.setLeafSize(0.3, 0.3, 0.3);
            filter.setInputCloud(node->FullPoints);
            filter.filter(cloud);
            pcl::toROSMsg(cloud, cloud_msg);
            cloud_msg.header.frame_id = lidar_link_frame_id;
            cloud_msg.header.stamp = node->Stamp;
            _pub_cloud_registered.publish(cloud_msg);
        }

        const auto ros_now = ros::Time::now();
        if (_local_map.empty() || (ros_now - last_time).toSec() < 1)
            return;
        last_time = ros_now;

        if (_pub_cloud_surround.getNumSubscribers() > 0)
        {
            pcl::toROSMsg(*_local_map.getFullPoints(), cloud_msg);
            cloud_msg.header.frame_id = local_map_frame_id;
            cloud_msg.header.stamp = _local_map.getStamp();
            _pub_cloud_surround.publish(cloud_msg);
        }

        if (_pub_cloud_surround_sharp.getNumSubscribers() > 0)
        {
            pcl::toROSMsg(*_local_map.getSharpFeatures(), cloud_msg);
            cloud_msg.header.frame_id = local_map_frame_id;
            cloud_msg.header.stamp = _local_map.getStamp();
            _pub_cloud_surround_sharp.publish(cloud_msg);
        }

        if (_pub_cloud_surround_flat.getNumSubscribers() > 0)
        {
            pcl::toROSMsg(*_local_map.getFlatFeatures(), cloud_msg);
            cloud_msg.header.frame_id = local_map_frame_id;
            cloud_msg.header.stamp = _local_map.getStamp();
            _pub_cloud_surround_flat.publish(cloud_msg);
        }
    }

    void _update_global_tf_tree()
    {
        static geometry_msgs::TransformStamped tf_msg;
        static tf::Transform T;

        tf_msg.header.frame_id = map_frame_id;
        tf_msg.child_frame_id = local_map_frame_id;

        tf::poseEigenToTF(T_map_local_map, T);
        tf::transformTFToMsg(T, tf_msg.transform);
        send_static_transform(tf_msg);
        tf_msg.header.frame_id = earth_frame_id;
        tf_msg.child_frame_id = map_frame_id;
#ifdef USE_GPSAutoAlignFactor3D
        tf::poseEigenToTF(_T_earth_map, T);
#else
        Pose3 temp;
        temp.setIdentity();
        temp.matrix().topLeftCorner(2, 2) = _T_earth_map.linear();
        temp.matrix().topRightCorner(2, 1) = _T_earth_map.translation();
        tf::poseEigenToTF(temp, T);
#endif
        tf::transformTFToMsg(T, tf_msg.transform);
        send_static_transform(tf_msg);
    }

    void _pub_loop_closure()
    {
        if (_loops.empty())
            return;

        auto loop_marker = boost::make_shared<visualization_msgs::MarkerArray>();
        size_t loop_count = _loops.size();
        loop_marker->markers.resize(loop_count + 1);
        loop_marker->markers[0].id = 0;
        loop_marker->markers[0].action = visualization_msgs::Marker::DELETEALL;

        for (size_t i = 0; i < loop_count; ++i)
        {
            auto& marker = loop_marker->markers[i + 1];
            marker.id = i + 1;
            marker.action = visualization_msgs::Marker::ADD;
            marker.header.frame_id = map_frame_id;
            marker.color.r = 0.8; marker.color.g = 0.1; marker.color.b = 0.1; marker.color.a = 1.0;
            marker.scale.x = 0.2; marker.scale.y = marker.scale.z = 0;
            marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0;
            marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0; marker.pose.orientation.w = 1;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.points.resize(2);
            tf::pointEigenToMsg(_global_map.at(_loops[i].first)->getCenter().translation(), marker.points[0]);
            tf::pointEigenToMsg(_global_map.at(_loops[i].second)->getCenter().translation(), marker.points[1]);           
        }
        _pub_loop_closure_marker.publish(loop_marker);
    }

    void _update_key_frame_path_global() const
    {
        if (_pub_key_frame_path_global.getNumSubscribers() <= 0)
            return;
        static nav_msgs::Path path;
        path.header.frame_id = map_frame_id;
        path.header.stamp = ros::Time::now();
        const auto size = _global_map.size();
        path.poses.resize(size);
        for (size_t i = 0; i < size; ++i)
        {
            const auto &node = _global_map.at(i);
            const auto &center = node->getCenter();
            Eigen::Quaterniond q(center.linear());
            path.poses[i].header.stamp = node->Stamp;
            path.poses[i].header.frame_id = map_frame_id;
            path.poses[i].pose.position.x = center.translation().x();
            path.poses[i].pose.position.y = center.translation().y();
            path.poses[i].pose.position.z = center.translation().z();
            path.poses[i].pose.orientation.w = q.w();
            path.poses[i].pose.orientation.x = q.x();
            path.poses[i].pose.orientation.y = q.y();
            path.poses[i].pose.orientation.z = q.z();
        }
        _pub_key_frame_path_global.publish(path);
    }

    void _pub_map_without_global_optimize(const MapNode::Ptr &node)
    {
        static CloudTypePtr map(new CloudType);
        static ros::Time last_time(0);
        static pcl::VoxelGrid<PointType> filter;

        CloudType temp;
        pcl::transformPointCloud(*node->FullPoints, temp, node->Pose.matrix());
        *map += temp;

        ros::Time now = node->Stamp;
        if ((now - last_time).toSec() < 3)
            return;
        last_time = now;

        apply_filter(filter, _full_leaf_size_global, map, map);

        if (_pub_cloud_map.getNumSubscribers() > 0)
        {
            sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*map, *msg);
            msg->header.frame_id = local_map_frame_id;
            msg->header.stamp = now;
            _pub_cloud_map.publish(msg);
        }
    }

    void _assosiate_transform(Pose3 &pose, const Pose3 &transform) const
    {
        pose = transform * pose;
        normalize_pose(pose);
    }

    void _update_transform(const Pose3 &pose, const Pose3 &pose_opt, Pose3 &transform) const
    {
        transform = pose_opt * pose.inverse();
        normalize_pose(transform);
    }

    void _send_key_frame(const MapNode::ConstPtr &latest_key_frame)
    {
        std::lock_guard<std::mutex> lock_guard(_m_key_frame_buf);
        _key_frame_buf.push_back(latest_key_frame);
    }

    void _copy_latest_key_frame(std::vector<MapNode::ConstPtr> &new_key_frames)
    {
        new_key_frames.clear();
        std::lock_guard<std::mutex> lock_guard(_m_key_frame_buf);
        new_key_frames.swap(_key_frame_buf);
    }

    void _send_latest_map_poses(const bool refresh_flag)
    {
        std::vector<Pose3> map_poses;
        for (const auto &node : _global_map)
        {
            map_poses.push_back(node->Pose);
        }
        std::lock_guard<std::mutex> lock_guard(_m_pub_global_map);
        _refresh_flag |= refresh_flag;
        _latest_map_poses.swap(map_poses);
    }

    void _copy_latest_map_poses(std::vector<Pose3> &map_poses, bool &refresh_flag)
    {
        refresh_flag = false;
        std::lock_guard<std::mutex> lock_guard(_m_pub_global_map);
        if (_latest_map_poses.empty())
            return;
        refresh_flag = _refresh_flag;
        _refresh_flag = false;
        map_poses.swap(_latest_map_poses);
        _latest_map_poses.clear();
    }

    void _cloud_info_msg_handler(const wloam::CloudInfoConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock_guard(_mBuf);
        _msg_buf.push_back(msg);
    }

    void _gps_odom_msg_handler(const nav_msgs::OdometryConstPtr &msg)
    {
        _update_gps_path_measured(msg->pose.pose.position.x , msg->pose.pose.position.y, msg->pose.pose.position.z, ROS_TIME(msg));

        std::lock_guard<std::mutex> lock_guard(_m_gps_msg_buf);
        _gps_msg_buf.push_back(msg);
    }

    Pose3 T_local_map_odom, T_map_local_map, _last_key_pose_in_local_map;

    std::shared_ptr<gtsam::ISAM2> _isam_global;

    Pose3 _T_lidar_gps;
#ifdef USE_GPSAutoAlignFactor3D
    Pose3 _T_earth_map;
#else
    Pose2 _T_earth_map;
#endif

    double _min_noise_prior;
    double _edge_noise_threshold;
    double _plane_noise_threshold;
    double _gps_noise_xy, _gps_noise_z;

    double _corner_leaf_size_local, _surf_leaf_size_local, _full_leaf_size_local;
    double _corner_leaf_size_global, _surf_leaf_size_global, _full_leaf_size_global;
    double _sampling_rate_local;
    double _sampling_rate_global;

    int _neighbor_range_for_loop;

    LocalMap _local_map;

    GlobalMap _global_map;

    gtsam::noiseModel::mEstimator::Huber::shared_ptr RobustLossFcn;

    std::vector<std::pair<size_t, size_t>> _loops;

    std::thread _local_optimize_thread, _global_optimize_thread, _global_map_pub_thread;
    std::mutex _mBuf, _m_key_frame_buf, _m_pub_global_map, _m_gps_msg_buf;
    std::deque<wloam::CloudInfoConstPtr> _msg_buf;
    std::deque<nav_msgs::OdometryConstPtr> _gps_msg_buf;
    std::vector<MapNode::ConstPtr> _key_frame_buf;
    std::vector<Pose3> _latest_map_poses;
    pcl::PointCloud<GPSPoint> _gps_path;
    bool _refresh_flag = false;

    ros::Subscriber _sub_msg, _sub_gps_odom;
    ros::Publisher _pub_cloud_registered, _pub_cloud_surround, _pub_cloud_surround_sharp, _pub_cloud_surround_flat, _pub_icp_registered_frame, _pub_cloud_map, _pub_selected_corner, _pub_selected_surf, _pub_selected_degenerate, _pub_gps_path_measured, _pub_key_frame_path_global, _pub_loop_closure_marker, _pub_optimized_path_global, _pub_odom_opt;

    const gtsam::Symbol _gps_node_id = G(0);

    bool _enable_global_optimize = false;
    bool _enable_loop_closure = false;
    bool _enable_gps_factor = false;
    bool _save_path = false;
    bool _save_pcd_map = false;
    bool _save_pcd_feature_map = false;
    std::atomic_bool _use_degenerate_feature;
    std::string _saving_path;
    double _key_frame_res_global;
    double _loop_closure_search_radius;
    double _min_feature_count;
    double _icp_fit_score_threshold;
    Pose3Covariance _prior_between_noise;

}; // ApaMapping

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ApaMapping");
    ros::NodeHandle nh;

    ApaMapping mapping;

    mapping.start_process();

    ros::spin();
}