#include <mutex>
#include <vector>
#include <queue>
#include <thread>
#include <string>
#include <Eigen/Core>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <apa_msgs/VehicleOdomEst2D.h>
#include "utils/common_names.h"
#include "utils/stampTools.hpp"
#include "utils/tfTools.hpp"
#include "utils/stampTools.hpp"

template <typename T>
class OdometryPublisherBase
{
public:
    using OdomT = T;

    OdometryPublisherBase():_last_odom_stamp(0), _last_twist_stamp(0)
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        _target_frame = private_nh.param<std::string>("correction_target_frame", local_map_frame_id);
        _source_frame = private_nh.param<std::string>("correction_source_frame", odom_frame_id);
        _child_frame= private_nh.param<std::string>("offset_source_frame", base_link_frame_id);
        const std::string odometer_frame = private_nh.param<std::string>("offset_target_frame", base_link_frame_id);
        const std::string sub_odom_topic =  topic_odometer + std::string("/") + odometer_frame;
        const std::string sub_twist_topic = topic_velometer + std::string("/") + odometer_frame;
        const std::string pub_topic_postfix = private_nh.param<std::string>("pub_topic_postfix", _target_frame + std::string("/") + _child_frame);

        _T_self_child = read_static_tf(odometer_frame, _child_frame); 
        
        Eigen::Isometry3d T_child_self;
    
        tf::transformTFToEigen(_T_self_child.inverse(), T_child_self);
        
        Eigen::Vector3d t(T_child_self.translation());
        Eigen::Matrix3d tx, R;
        R = T_child_self.linear();
        tx <<   0.0, -t.z(),  t.y(),
              t.z(),    0.0, -t.x(), 
             -t.y(),  t.x(),    0.0;
             
        _adj_child_self << R, Eigen::Matrix3d::Zero(), tx * R, R;
        
        if (_target_frame != "" &&  !_recvT(_target_frame, _source_frame, ros::Time(0), _T_target_source, 5))
        {
            ROS_FATAL("no Transform found from %s to %s within 5 seconds", _source_frame.c_str(), _target_frame.c_str());
            return;
        }          

        _pub_odom_info = nh.advertise<nav_msgs::Odometry>(topic_odometer + std::string("/") + pub_topic_postfix, 4);
        _pub_twist_info = nh.advertise<geometry_msgs::TwistStamped>(topic_velometer + std::string("/") + _child_frame + "_local", 4);
        _sub_odom_info = nh.subscribe<OdomT>(sub_odom_topic, 4, &OdometryPublisherBase::_odom_info_handler, this);
        _sub_twist_info = nh.subscribe<geometry_msgs::TwistStamped>(sub_twist_topic, 4, &OdometryPublisherBase::_pub_twist, this);
    }

    virtual ~OdometryPublisherBase() {}

protected:

    void _pub_odom(const nav_msgs::Odometry::Ptr& odom)
    {
        if (ROS_TIME(odom) <= _last_odom_stamp)
            return;

        _last_odom_stamp = ROS_TIME(odom);

        if (_pub_odom_info.getNumSubscribers() < 1)
            return;

        if (_target_frame != "")
            _recvT(_target_frame, _source_frame, ros::Time(0), _T_target_source);

        tf::Transform T_odom;
        tf::poseMsgToTF(odom->pose.pose, T_odom);
        tf::poseTFToMsg(_T_target_source * T_odom * _T_self_child, odom->pose.pose);

        odom->header.frame_id = _target_frame;
        odom->child_frame_id = _child_frame;

        _pub_odom_info.publish(odom);
    }

    void _pub_twist(const geometry_msgs::TwistStampedConstPtr& twist_msg)
    {
        if(ROS_TIME(twist_msg) <= _last_twist_stamp)
            return;

        _last_twist_stamp = ROS_TIME(twist_msg);

        if(_pub_twist_info.getNumSubscribers() < 1)
            return;

        Eigen::Matrix<double, 6, 1> twist;
        twist << twist_msg->twist.angular.x, twist_msg->twist.angular.y, twist_msg->twist.angular.z,
                 twist_msg->twist.linear.x,  twist_msg->twist.linear.y,  twist_msg->twist.linear.z;
        twist = _adj_child_self * twist;
        geometry_msgs::TwistStamped msg;
        msg.header.stamp = twist_msg->header.stamp;
        msg.header.frame_id = _child_frame;
        msg.twist.angular.x = twist[0];
        msg.twist.angular.y = twist[1];
        msg.twist.angular.z = twist[2];
        msg.twist.linear.x = twist[3];
        msg.twist.linear.y = twist[4];
        msg.twist.linear.z = twist[5];
        _pub_twist_info.publish(msg);
    }

    bool _recvT(const std::string &target_frame, const std::string &source_frame, const ros::Time &time, tf::StampedTransform &transform, const double wait_time = 0) const
    {
        try
        {
            _listener.waitForTransform(target_frame, source_frame, time, ros::Duration(wait_time));
            _listener.lookupTransform(target_frame, source_frame, time, transform);
        }
        catch (const tf::TransformException &e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        return true;
    }

    virtual void _odom_info_handler(const typename OdomT::ConstPtr &odom_info) = 0;

private:
    Eigen::Matrix<double, 6, 6> _adj_child_self;
    ros::Subscriber _sub_odom_info, _sub_twist_info;
    ros::Publisher _pub_odom_info, _pub_twist_info;
    tf::TransformListener _listener;
    tf::StampedTransform _T_target_source,  _T_self_child;;
    std::string _target_frame, _source_frame, _child_frame;
    ros::Time _last_odom_stamp, _last_twist_stamp;

}; // class OdometryPublisherBase


class PoseStampedPublisher : public OdometryPublisherBase<geometry_msgs::PoseStamped>
{

protected:
    virtual void _odom_info_handler(const OdomT::ConstPtr &odom_info) override
    {
        nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
        odom->header.stamp = odom_info->header.stamp;
        odom->pose.pose = odom_info->pose;
        _pub_odom(odom);
    }
};

class OdometryPublisher : public OdometryPublisherBase<nav_msgs::Odometry>
{
protected:
    virtual void _odom_info_handler(const OdomT::ConstPtr &odom_info) override
    {
        _pub_odom(boost::make_shared<OdomT>(*odom_info));
    }
}; // class OdometryPublisher

class ApaOdometryPublisher : public OdometryPublisherBase<apa_msgs::VehicleOdomEst2D>
{
protected:
    virtual void _odom_info_handler(const OdomT::ConstPtr &odom_info) override
    {
        nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry());
        odom->header.stamp = odom_info->header.stamp;
        odom->pose.pose.position.x = odom_info->x;
        odom->pose.pose.position.y = odom_info->y;
        odom->pose.pose.position.z = 0;
        const auto q = tf::createQuaternionFromYaw(odom_info->yaw);
        odom->pose.pose.orientation.w = q.w();
        odom->pose.pose.orientation.x = q.x();
        odom->pose.pose.orientation.y = q.y();
        odom->pose.pose.orientation.z = q.z();
        _pub_odom(odom);
    }

}; // class ApaOdometryPublisher

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Time::waitForValid();

    PoseStampedPublisher odom_pub;
    
    ros::spin();

    return 0;
}