#include <cmath>
#include <string>
#include <fstream>
#include <array>

#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include "utils/frame_id.h"

tf::TransformListener *tf_listener;

bool get_self_rect(const std::string &param_json_file, const std::string &frame_id, const std::array<float, 4> &rgba, visualization_msgs::Marker &self_marker)
{    
    Json::Value params;
    Json::Reader param_reader;
    std::ifstream fs(param_json_file);
    if (!param_reader.parse(fs, params, false))
    {
        std::cerr << "params file opening error in Visualization" << std::endl;
        fs.close();
        return false;
    }
    fs.close();

    const auto length = params.get("length", -1).asDouble();
    const auto width = params.get("width", -1).asDouble();
    const auto rear_overhang = params.get("rear_overhang", -1).asDouble();

    if (length == -1 || width == -1 || rear_overhang == -1)
        return false;

    self_marker.points.clear();
    self_marker.header.frame_id = frame_id;
    self_marker.header.seq = 0;
    self_marker.id = 0;

    // self_marker.frame_locked = 1;
    // self_marker.action = visualization_msgs::Marker::MODIFY;
    self_marker.color.r = rgba[0];
    self_marker.color.g = rgba[1];
    self_marker.color.b = rgba[2];
    self_marker.color.a = rgba[3];
    self_marker.scale.x = 0.1;
    self_marker.pose.orientation.w = 1;
    self_marker.pose.orientation.x = 0;
    self_marker.pose.orientation.y = 0;
    self_marker.pose.orientation.z = 0;
    self_marker.type = visualization_msgs::Marker::LINE_LIST;
    self_marker.ns = "car_rect";

    geometry_msgs::Point fl, fr, rl, rr;
    fl.x = length - rear_overhang;
    fl.y = width / 2;
    fl.z = 0;

    rr.x = -rear_overhang;
    rr.y = -width / 2;
    rr.z = 0;

    fr.x = fl.x;
    fr.y = -fl.y;
    fr.z = 0;

    rl.x = rr.x;
    rl.y = -rr.y;
    rl.z = 0;

    self_marker.points.emplace_back(fl);
    self_marker.points.emplace_back(rl);
    self_marker.points.emplace_back(rl);
    self_marker.points.emplace_back(rr);
    self_marker.points.emplace_back(rr);
    self_marker.points.emplace_back(fr);
    self_marker.points.emplace_back(fr);
    self_marker.points.emplace_back(fl);

    return true;
}

void visualize_path(const std::string &target_frame_id, const std::string &source_frame_id, nav_msgs::Path &path, const ros::Publisher &pub)
{
    tf::StampedTransform transform;
    try
    {
        tf_listener->lookupTransform(target_frame_id, source_frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        // ROS_INFO("%s\nin visulization\n", ex.what());
        return;
    }

    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.header.stamp = transform.stamp_;
    tf::poseTFToMsg(transform, pose.pose);
    if (path.poses.size() > 0)
    {
        const double x_diff = path.poses.back().pose.position.x - pose.pose.position.x;
        const double y_diff = path.poses.back().pose.position.y - pose.pose.position.y;
        const double z_diff = path.poses.back().pose.position.z - pose.pose.position.z;
        if (x_diff * x_diff + y_diff * y_diff + z_diff * z_diff < 0.1 * 0.1)
            return;
    }
    path.poses.push_back(std::move(pose));
    if(pub.getNumSubscribers() > 0)
        pub.publish(path);
}

void visualize_self_rec(visualization_msgs::Marker& self_rect, const ros::Publisher &pub)
{
    self_rect.header.stamp = ros::Time::now();
    self_rect.header.seq++;
    self_rect.id = 0;
    self_rect.frame_locked = 1;
    self_rect.action = visualization_msgs::Marker::MODIFY;
    self_rect.lifetime = ros::Duration(0);
    pub.publish(self_rect);
}

void visualize_self_rec_history(visualization_msgs::Marker& self_rect, const ros::Publisher &pub)
{
    self_rect.id++;
    self_rect.header.seq++;
    self_rect.header.stamp = ros::Time::now();
    self_rect.frame_locked = 0;
    self_rect.action = visualization_msgs::Marker::ADD;
    self_rect.lifetime = ros::Duration(30);
    pub.publish(self_rect);
}

int main(int argc, char **argv)
{
    const int loop_rate = 30;
    ros::init(argc, argv, "visulization");
    ros::NodeHandle nh;
    ros::Rate rate(loop_rate);

    tf::TransformListener listener;
    tf_listener = &listener;

    ros::Publisher pub_path_by_lidar = nh.advertise<nav_msgs::Path>("path_by_lidar", 2);

    ros::Publisher pub_self_rec = nh.advertise<visualization_msgs::Marker>("self_rect_marker", 1);

    ros::Publisher pub_self_rec_history = nh.advertise<visualization_msgs::Marker>("self_rect_marker_history", 1);

    std::string param_json_file;
    nh.param<std::string>("meta_vehilce_file_path", param_json_file, "./src/wloam/vehicle_params/E50.json");

    visualization_msgs::Marker self_marker_base, self_marker_base_history;
    
    if(!get_self_rect(param_json_file, base_link_frame_id, {0,1,0,1} ,self_marker_base))
    {
        ROS_ERROR("visualization: get rect param fail\n");
        return EXIT_FAILURE;
    }
    self_marker_base_history = self_marker_base;

    nav_msgs::Path path_by_lidar;

    path_by_lidar.header.frame_id = map_frame_id;

    uint8_t count = 0;
    while (nh.ok())
    {
        if (count++ == loop_rate)
        {
            count = 0;
            visualize_self_rec(self_marker_base, pub_self_rec);
            visualize_self_rec_history(self_marker_base_history, pub_self_rec_history);
        }
        visualize_path(map_frame_id, base_link_frame_id, path_by_lidar, pub_path_by_lidar);
        rate.sleep();
    }
}