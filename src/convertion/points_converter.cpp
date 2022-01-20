#include <algorithm>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <utils/my_point_type.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include "utils/stampTools.hpp"
#include "utils/common_names.h"


using CloudType = wloam::CloudType;

class PointCloud2Converter
{

public:
    enum Format
    {
        UNKOWN,
        VELODYNE,
        OUSTER,
        ROBOSENSE
    };
    using Format = PointCloud2Converter::Format;

    void setFormat(Format format) { _format = format; }

    void convert(const sensor_msgs::PointCloud2ConstPtr &msg, CloudType &cloud, ros::Time &fixed_stamp, const bool re_sort) const
    {
        cloud.clear();
        switch (_format)
        {
        case VELODYNE:
            _convert_velodyne(msg, cloud);
            break;
        case OUSTER:
            _convert_ouster(msg, cloud);
            break;
        case ROBOSENSE:
            _convert_robosense(msg, cloud);
            break;
        default:
            _convert_default(msg, cloud);
            break;
        }
        if (cloud.empty())
            return;
        if(re_sort)
            _sort_cloud(cloud);
        // _downsample(cloud, 1);
        fixed_stamp = ROS_TIME(msg);
        _fix_timestamp(fixed_stamp, cloud);
    }

private:
    void _downsample(CloudType& cloud, const int skip) const
    {
        int j = 0;
        const auto size = cloud.size();
        for(int i = 0; i < size; i+= skip + 1)
        {
            cloud[j++] = cloud[i];
        }
        cloud.resize(j);
    }

    template <typename PointT>
    void remove_bad_points(pcl::PointCloud<PointT> &cloud) const
    {
        size_t i = 0;
        for(const auto &p : cloud)
        {
            if(!(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))) continue;
            if(p.x * p.x + p.y * p.y + p.z * p.z == 0.0) continue;
            cloud[i++] = p;
        }

        if(i != cloud.size())
        {
            cloud.resize(i);
        }

        cloud.is_dense = true;
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(i);
    }    

    void _fix_timestamp(ros::Time &stamp, CloudType &cloud) const
    {
        float rel_t0 = std::min_element(cloud.begin(), cloud.end(), [](const CloudType::PointType&u, const CloudType::PointType& v){return u.rel_time < v.rel_time;})->rel_time;
        stamp += ros::Duration(rel_t0);
        cloud.header.stamp = pcl_conversions::toPCL(stamp);
        for (auto &p : cloud)
        {
            p.rel_time -= rel_t0;
        }
    }

    void _convert_default(const sensor_msgs::PointCloud2ConstPtr &msg, CloudType &cloud) const
    {
        pcl::fromROSMsg(*msg, cloud);
        remove_bad_points(cloud);
    }

    void _convert_velodyne(const sensor_msgs::PointCloud2ConstPtr &msg, CloudType &cloud) const
    {
        _convert_default(msg, cloud);
    }

    void _convert_ouster(const sensor_msgs::PointCloud2ConstPtr &msg, CloudType &cloud) const
    {
        pcl::PointCloud<wloam::OusterPoint> os_cloud;
        pcl::fromROSMsg(*msg, os_cloud);
        remove_bad_points(os_cloud);
        _copy_meta_and_resize(os_cloud, cloud);
        const auto size = os_cloud.size();
        for(int i = 0; i < size; ++i)
        {
            const auto& p_in = os_cloud[i];
            auto& p_out = cloud[i];
            _copy_xyzi(p_in, p_out);
            p_out.ring = p_in.ring;
            p_out.rel_time = p_in.t * 1e-9;
        }
    }

    void _convert_robosense(const sensor_msgs::PointCloud2ConstPtr &msg, CloudType &cloud) const
    {
        pcl::PointCloud<wloam::RsPointXYZIRT> rs_cloud;
        pcl::fromROSMsg(*msg, rs_cloud);
        remove_bad_points(rs_cloud);
        _copy_meta_and_resize(rs_cloud, cloud);
        const auto size = rs_cloud.size();
        const double t_base = msg->header.stamp.toSec();
        for (int i = 0; i < size; ++i)
        {
            const auto &p_in = rs_cloud[i];
            auto &p_out = cloud[i];
            _copy_xyzi(p_in, p_out);
            p_out.ring = p_in.ring;
            p_out.rel_time = p_in.timestamp - t_base;
        }
    }

    template <typename T>
    void _copy_xyzi(const T &p_in, CloudType::PointType &p_out) const
    {
        p_out.x = p_in.x;
        p_out.y = p_in.y;
        p_out.z = p_in.z;
        p_out.intensity = p_in.intensity;
    }

    template <typename T>
    void _copy_meta_and_resize(const T &in, CloudType &out) const
    {
        out.header = in.header;
        out.width = in.width;
        out.height = in.height;
        out.is_dense = in.is_dense;
        out.resize(in.size());
    }

    void _sort_cloud(CloudType &cloud) const
    {
        std::sort(cloud.begin(), cloud.end(), [](const CloudType::PointType &u, const CloudType::PointType &v) {
            if(u.ring != v.ring) return u.ring < v.ring;
            return u.rel_time < v.rel_time;
        });
    }

    Format _format = UNKOWN;

}; // class PointCloud2Converter

PointCloud2Converter converter;
ros::Publisher pub;
bool re_sort;

void cloud_handler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    static sensor_msgs::PointCloud2 msg_out;
    ros::Time stamp;
    CloudType cloud_out;
    converter.convert(msg, cloud_out, stamp, re_sort);
    pcl::toROSMsg(cloud_out, msg_out);
    msg_out.header.frame_id = lidar_link_frame_id;
    msg_out.header.stamp = stamp;
    pub.publish(msg_out);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "points_converter");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    const auto in = private_nh.param<std::string>("points_topic", "velodyne_points");
    const std::string cloud_msg_format = private_nh.param<std::string>("cloud_msg_format", "");
    re_sort = private_nh.param<bool>("re_sort", true);
    if(re_sort)
        ROS_INFO("enable re_sort");
    else
        ROS_INFO("disable re_sort");
    if (cloud_msg_format == "velodyne")
        converter.setFormat(PointCloud2Converter::Format::VELODYNE);
    else if (cloud_msg_format == "ouster")
        converter.setFormat(PointCloud2Converter::Format::OUSTER);
    else if (cloud_msg_format == "robosense")
        converter.setFormat(PointCloud2Converter::Format::ROBOSENSE);
    else
        converter.setFormat(PointCloud2Converter::Format::UNKOWN);
    
    ROS_INFO("convert topic /%s(%s) to /%s(PointXYZIRT)", in.c_str(), cloud_msg_format.c_str(), topic_laser_cloud_full);
    
    pub = nh.advertise<sensor_msgs::PointCloud2>(topic_laser_cloud_full, 10);
    ros::Subscriber sub_velo = nh.subscribe<sensor_msgs::PointCloud2>(in, 10, cloud_handler, ros::TransportHints().tcpNoDelay());

    ros::spin();

}

