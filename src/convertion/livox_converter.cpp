#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <utils/my_point_type.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include "utils/stampTools.hpp"
#include "utils/common_names.h"
#include <wloam/CustomMsg.h>
#include <sensor_msgs/Imu.h>

using namespace wloam;
constexpr double G = 9.8099;

ros::Publisher pub_cloud, pub_imu;

void imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
    sensor_msgs::Imu msg_out = *msg;
    msg_out.linear_acceleration.x *= G;
    msg_out.linear_acceleration.y *= G;
    msg_out.linear_acceleration.z *= G;
    msg_out.header.frame_id = imu_link_frame_id;
    pub_imu.publish(msg_out);
}

void cloud_callback(const CustomMsgConstPtr& msg)
{
    sensor_msgs::PointCloud2 msg_out;
    CloudType cloud;
    cloud.resize(msg->point_num);
    for(int i = 0; i < msg->point_num; ++i)
    {
        cloud[i].x = msg->points[i].x;
        cloud[i].y = msg->points[i].y;
        cloud[i].z = msg->points[i].z;
        cloud[i].intensity = msg->points[i].reflectivity;
        cloud[i].ring = msg->points[i].line;
        cloud[i].rel_time = 1e-9 * msg->points[i].offset_time;
    }
    pcl::toROSMsg(cloud, msg_out);
    msg_out.header.stamp.fromNSec(msg->timebase);
    msg_out.header.frame_id = lidar_link_frame_id; 
    pub_cloud.publish(msg_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "livox_converter");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    const auto sub_cloud_topic = private_nh.param<std::string>("sub_cloud_topic", "livox/lidar");
    const auto sub_imu_topic = private_nh.param<std::string>("sub_imu_topic", "livox/imu");
    const auto pub_cloud_topic = private_nh.param<std::string>("pub_cloud_topic", "points_raw");
    const auto pub_imu_topic = private_nh.param<std::string>("pub_imu_topic", topic_imu_msg);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(pub_cloud_topic, 8);
    pub_imu = nh.advertise<sensor_msgs::Imu>(pub_imu_topic, 64);
    const auto sub_cloud = nh.subscribe<CustomMsg>(sub_cloud_topic, 10, cloud_callback);
    const auto sub_imu = nh.subscribe<sensor_msgs::Imu>(sub_imu_topic, 64, imu_callback);
    ROS_INFO("PointCloud: Subscribt to %s, Publish to %s", sub_cloud_topic.c_str(), pub_cloud_topic.c_str());
    ROS_INFO("IMU: Subscribt to %s, Publish to %s", sub_imu_topic.c_str(), pub_imu_topic.c_str());

    ros::spin();
}
