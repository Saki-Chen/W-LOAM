#include <thread>
#include <algorithm>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <apa_msgs/WheelEncoderStamped.h>
#include <apa_msgs/SteeringAngleStamped.h>
#include <apa_msgs/ChasisSpeedStamped.h>
#include <apa_msgs/VehicleOdomEst2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>

#include "wheel_odometry.h"
#include "utils/common_names.h"

bool is_odometer;
tf::TransformBroadcaster* tf_broadcaster;

odometry::WheelOdometry* wheel_odom;

ros::Publisher pub_vehicle_odom;
ros::Publisher pub_wheel_odom, pub_wheel_twist;

void is_forward_handler(const apa_msgs::ChasisSpeedStampedConstPtr &speed)
{
    wheel_odom->updateDirection(speed->MotorSpeed>0);
}

void wheeling_count_handler(const apa_msgs::WheelEncoderStampedConstPtr &wheeling_count)
{
    wheel_odom->updateWheelingCount(wheeling_count->FL, wheeling_count->FR, wheeling_count->RL, wheeling_count->RR, wheeling_count->header.stamp.toNSec());
}

void publish_DR()
{
    uint64_t stamp, twist_stamp;
    tf::StampedTransform T_odom_base;
    double x, y, yaw, distance, local_v, local_yaw_rate;
    bool is_new_pose = wheel_odom->isNewPose();
    bool is_new_twist = wheel_odom->isNewTwist();
    wheel_odom->getPose(x, y, yaw, distance, stamp);
    wheel_odom->getTwist(local_v, local_yaw_rate, twist_stamp);

    const auto q = tf::Quaternion(tf::Vector3(0,0,1), yaw);
    const auto t = tf::Vector3(x, y, 0);

    if(is_new_twist)
    {
        geometry_msgs::TwistStamped wheel_twist_msg;
        wheel_twist_msg.header.frame_id = odom_frame_id;
        wheel_twist_msg.header.stamp.fromNSec(twist_stamp);
        wheel_twist_msg.twist.linear.x = local_v;
        wheel_twist_msg.twist.linear.y = wheel_twist_msg.twist.linear.z = 0;
        wheel_twist_msg.twist.angular.z = local_yaw_rate;
        wheel_twist_msg.twist.angular.x = wheel_twist_msg.twist.angular.y = 0;
        pub_wheel_twist.publish(wheel_twist_msg);
    }

    if(is_new_pose)
    {
        geometry_msgs::PoseStamped wheel_odom_msg;
        wheel_odom_msg.header.frame_id = odom_frame_id;
        wheel_odom_msg.header.stamp.fromNSec(stamp);
        wheel_odom_msg.pose.position.x = t.x();
        wheel_odom_msg.pose.position.y = t.y();
        wheel_odom_msg.pose.position.z = t.z();
        wheel_odom_msg.pose.orientation.w = q.w();
        wheel_odom_msg.pose.orientation.x = q.x();
        wheel_odom_msg.pose.orientation.y = q.y();
        wheel_odom_msg.pose.orientation.z = q.z();

        pub_wheel_odom.publish(wheel_odom_msg);

        apa_msgs::VehicleOdomEst2D pose2D;
        pose2D.header.frame_id = odom_frame_id;
        pose2D.header.stamp.fromNSec(stamp);
        pose2D.x = x;
        pose2D.y = y;
        pose2D.yaw = yaw;
        pose2D.distance = distance;

        pose2D.twist_stamp.fromNSec(twist_stamp);
        pose2D.yaw_rate = local_yaw_rate;
        pose2D.linear_speed = local_v;
        pub_vehicle_odom.publish(pose2D);

        if(!is_odometer) return;

        T_odom_base.setOrigin(t);
        T_odom_base.setRotation(q); 
        T_odom_base.frame_id_ = odom_frame_id;
        T_odom_base.child_frame_id_ = base_link_frame_id;
        T_odom_base.stamp_.fromNSec(stamp);
        tf_broadcaster->sendTransform(T_odom_base);
    }
}

void steering_angle_handler(const apa_msgs::SteeringAngleStampedConstPtr &angle)
{
    wheel_odom->updateSteeringAngle(-angle->angle);
    if(wheel_odom->isReady())
        publish_DR();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "DeadReckoning");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string param_json_file = nh.param<std::string>("meta_vehilce_file_path", "./src/wloam/vehicle_params/E50.json");
    is_odometer = (base_link_frame_id == nh.param<std::string>("odometer_link", ""));
    double sliding_window_len = private_nh.param<double>("sliding_window_len", 0.5);
    
    odometry::WheelOdometry odom(param_json_file);
    wheel_odom = &odom;
    wheel_odom->setSlidingWindow(sliding_window_len);

    tf::TransformBroadcaster br;
    tf_broadcaster = &br;
    const auto sub_steering_angle = nh.subscribe<apa_msgs::SteeringAngleStamped>(topic_steering_angle, 128, steering_angle_handler);
    const auto sub_speed = nh.subscribe<apa_msgs::ChasisSpeedStamped>(topic_chasis_speed, 128, is_forward_handler);
    const auto sub_wheeling_count = nh.subscribe<apa_msgs::WheelEncoderStamped>(topic_wheeling_count, 128, wheeling_count_handler);
    
    pub_vehicle_odom = nh.advertise<apa_msgs::VehicleOdomEst2D>(topic_extra_odom_info, 16);
    pub_wheel_odom = nh.advertise<geometry_msgs::PoseStamped>(topic_odometer + std::string("/") + base_link_frame_id, 16);
    pub_wheel_twist = nh.advertise<geometry_msgs::TwistStamped>(topic_velometer + std::string("/") + base_link_frame_id, 16);

    ros::spin();
}