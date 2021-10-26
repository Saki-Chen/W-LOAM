#include <string>
#include <iostream>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <gps_common/conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include "utils/tfTools.hpp"
#include "utils/stampTools.hpp"
#include "utils/common_names.h"

constexpr double deg_rad = M_PI / 180.0;
int gps_status_filter = -1;
tf::Pose T_gps_lidar;
ros::Publisher pub_gps_odom, pub_lidar_pose, pub_gps_fix;
std::vector<double> datum{-1, -1, -1};
std::array<double, 3> cartesian_ref;
std::string ref_utm_zone("");

static inline double getZoneMeridian(const std::string& utm_zone)
{
  int zone_number = std::atoi(utm_zone.substr(0,2).c_str());
  return (zone_number == 0) ? 0.0 : (zone_number - 1) * 6.0 - 177.0;
}

void call_back(const sensor_msgs::ImuConstPtr& imu_msg, const sensor_msgs::NavSatFixConstPtr& nav_msg)
{
    static ros::Time last_stamp(0);
    static tf::Pose transform;
    static nav_msgs::Odometry gps_odom, lidar_pose;
    if(ROS_TIME(nav_msg) <= last_stamp || nav_msg->status.status < 0 || (gps_status_filter != -1 && nav_msg->status.status != gps_status_filter)) return;
    last_stamp = ROS_TIME(nav_msg);

    if(ref_utm_zone == "")
    {
        if(datum[0] < 0)
        {
            datum[0] = nav_msg->latitude;
            datum[1] = nav_msg->longitude;
            datum[2] = nav_msg->altitude;
            ROS_INFO("set datum to latitude: %f, longititude: %f, altitude: %f", nav_msg->latitude, nav_msg->longitude, nav_msg->altitude);
        }
        gps_common::LLtoUTM(datum[0], datum[1], cartesian_ref[1], cartesian_ref[0], ref_utm_zone);
        cartesian_ref[2] = datum[2];
        ROS_INFO("Cartesian Referrence Origin is set to (%f, %f, %f) at utm zoom %s", cartesian_ref[0], cartesian_ref[1], cartesian_ref[2], ref_utm_zone.c_str());
    }

    double utm_x, utm_y;
    std::string utm_zone;
    gps_common::LLtoUTM(nav_msg->latitude, nav_msg->longitude, utm_y, utm_x, utm_zone);

    if(ref_utm_zone!= utm_zone)
    {
        ROS_ERROR("outside ref utm zone %s, current at utm zone %s", ref_utm_zone.c_str(), utm_zone.c_str());
        return;
    }

    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w)).getRPY(roll, pitch, yaw);
    double convergence_angle = std::atan(std::tan(deg_rad * (nav_msg->longitude - getZoneMeridian(utm_zone))) * std::sin(deg_rad * nav_msg->latitude));
    yaw += convergence_angle;
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    gps_odom.header.frame_id = earth_frame_id;
    gps_odom.header.stamp = ROS_TIME(nav_msg);
    gps_odom.child_frame_id = gps_link_frame_id;
    gps_odom.pose.pose.position.x = utm_x - cartesian_ref[0];
    gps_odom.pose.pose.position.y = utm_y - cartesian_ref[1];
    gps_odom.pose.pose.position.z = nav_msg->altitude - cartesian_ref[2];
    tf::quaternionTFToMsg(q, gps_odom.pose.pose.orientation);
    pub_gps_odom.publish(gps_odom);

    lidar_pose.header = gps_odom.header;
    lidar_pose.child_frame_id = lidar_link_frame_id;
    tf::poseMsgToTF(gps_odom.pose.pose, transform);
    transform *= T_gps_lidar;
    tf::poseTFToMsg(transform, lidar_pose.pose.pose);
    pub_lidar_pose.publish(lidar_pose);
}

void pub_correct_gps(const nav_msgs::OdometryConstPtr& gps_odom)
{
    if(ref_utm_zone == "") return;

    static sensor_msgs::NavSatFix gps_fix;
    double utm_x = gps_odom->pose.pose.position.x + cartesian_ref[0];
    double utm_y = gps_odom->pose.pose.position.y + cartesian_ref[1];
    double altitude = gps_odom->pose.pose.position.z + cartesian_ref[2];

    gps_fix.header.stamp = gps_odom->header.stamp;
    gps_fix.header.frame_id = gps_odom->child_frame_id;
    gps_fix.altitude = altitude;
    gps_common::UTMtoLL(utm_y, utm_x, ref_utm_zone.c_str(), gps_fix.latitude, gps_fix.longitude);
    pub_gps_fix.publish(gps_fix);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_odom_publisher");
    ros::NodeHandle nh, private_nh("~");
    ros::Time::waitForValid();
    gps_status_filter = private_nh.param<int>("gps_status_filter", -1);

    private_nh.param("datum", datum, {-1, -1, -1});
    if(datum[0] >= 0)
        ROS_INFO("set datum to latitude: %f, longititude: %f, altitude: %f", datum[0], datum[1], datum[2]);

    T_gps_lidar = read_static_tf(gps_link_frame_id, lidar_link_frame_id);
    
    pub_gps_odom = nh.advertise<nav_msgs::Odometry>(topic_gps_odom, 32);
    pub_lidar_pose = nh.advertise<nav_msgs::Odometry>(topic_odometer + std::string("/") + lidar_link_frame_id + "_GT", 32);
    pub_gps_fix = nh.advertise<sensor_msgs::NavSatFix>("gps/fix_correct", 32);

    ros::Subscriber sub_gps_odom = nh.subscribe<nav_msgs::Odometry>("/odometer/earth/gps_link", 128, pub_correct_gps);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, topic_imu_msg, 2048);
    message_filters::Subscriber<sensor_msgs::NavSatFix> nav_sub(nh, topic_gps_fix, 128);
    using Policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::NavSatFix>;
    message_filters::Synchronizer<Policy> sync(Policy(1024), imu_sub, nav_sub);
    sync.registerCallback(boost::bind(&call_back, _1, _2));
    
    ros::spin();
}