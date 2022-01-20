#include <mutex>
#include <queue>
#include <algorithm>
#include <thread>
#include <Eigen/Core>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <wloam/CloudInfo.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include "utils/common_names.h"
#include "utils/tfTools.hpp"
#include "utils/stampTools.hpp"
#include <utils/my_point_type.h>

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

using CloudType = wloam::CloudType;

class ScanExtraction
{
    struct PointInfo
    {
        float range;
        float angle_diff;
    };

    struct CloudExtractInfo
    {
        using Ptr = std::shared_ptr<CloudExtractInfo>;
        std_msgs::Header header;
        CloudType cloud;
        Eigen::Isometry3d pose;
        std::vector<char> label;
        std::vector<std::pair<int, int>> segments;

        void clear()
        {
            cloud.clear();
            label.clear();
            segments.clear();
        }
    }; // struct CloudExractInfo
public:
    ScanExtraction()
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        MIN_POINT_DIST = private_nh.param<double>("min_point_dist", 1);
        ENABLE_DESKEW = private_nh.param<bool>("enable_deskew", true);
        const auto odometer_link = private_nh.param<std::string>("odometer_link", base_link_frame_id);
        const auto P_odometer_lidar = read_static_tf(odometer_link, lidar_link_frame_id);
        Eigen::Isometry3d T_lidar_velometer;
        tf::poseTFToEigen(P_odometer_lidar, T_ODOMETER_LIDAR);

        SMOOTH_THRESHOLD = private_nh.param<float>("smooth_threshold_deg", 15.0) * M_PI / 180.0;
        SHARP_THRESHOLD = private_nh.param<float>("sharp_threshold_deg", 45.0) * M_PI / 180.0;
        NEIGHBOR_RADIUS = private_nh.param<int>("neighbor_search_radius", 5);
        ANGLE_DIFF_THREHOLD = private_nh.param<float>("angle_diff_threshold", 3) * M_PI / 180.0;
        DISTANCE_DIFF_THRESHOLD = private_nh.param<float>("distance_diff_threshold", 2);
        MIN_SEG_LEN = 5;

        _pub_feature_flat = nh.advertise<sensor_msgs::PointCloud2>(topic_laser_cloud_flat, 10);

        _pub_feature_sharp = nh.advertise<sensor_msgs::PointCloud2>(topic_laser_cloud_sharp, 10);

        _pub_cloud_info = nh.advertise<wloam::CloudInfo>(topic_cloud_info, 10);

        _pub_cloud_deskewed = nh.advertise<sensor_msgs::PointCloud2>(topic_laser_cloud_full + std::string("_deskewed"), 10);

        _pub_odom_lidar = nh.advertise<geometry_msgs::PoseStamped>(topic_odometry_lidar, 10);

        _sub_odom_msg = nh.subscribe<geometry_msgs::PoseStamped>(topic_odometer + std::string("/") + odometer_link, 4096, &ScanExtraction::_odom_msg_handler, this);

        _extraction_thread = std::thread(&ScanExtraction::_extraction_process, this);

        _deskew_thread = std::thread(&ScanExtraction::_deskew_process, this);

    }

    void SpinOnMultiQueue()
    {
        ros::NodeHandle nh;
        ros::CallbackQueue queue;
        nh.setCallbackQueue(&queue);
        _sub_cloud_msg = nh.subscribe<sensor_msgs::PointCloud2>(topic_laser_cloud_full, 32, &ScanExtraction::_cloud_msg_handler, this, ros::TransportHints().tcpNoDelay());
        ros::AsyncSpinner aspiner(1, &queue);
        aspiner.start();
        ros::spin();
        aspiner.stop();
    }

    ~ScanExtraction() {_extraction_thread.join(); _deskew_thread.join();}

private:
    void _extraction_process()
    {
        ros::NodeHandle nh;
        sensor_msgs::PointCloud2ConstPtr cloud_msg;
        std::vector<PointInfo> point_info;
        std::deque<CloudExtractInfo::Ptr> recycled;
        ros::Time t0;
        wait_and_pop_with_lock(_odom_msg_buf, _m_odom_msg_buf, ros::Time(0), 100, t0, nh);
        while (nh.ok())
        {
            wait_and_get_with_lock(_cloud_msg_buf, _m_cloud_msg_buf, t0, 100, cloud_msg, nh);

            CloudExtractInfo::Ptr extract_info;
            if(recycled.size() > 16 && recycled.front().use_count() < 2)
            {
                extract_info.swap(recycled.front());
                recycled.pop_front();
                extract_info->cloud.clear();
                extract_info->label.clear();
                extract_info->segments.clear();
                // ROS_INFO("use recycled, size %d", recycled.size());
            }
            else
            {
                extract_info = std::make_shared<CloudExtractInfo>();
                // ROS_INFO("make new");
            }
            extract_info->header = cloud_msg->header;
            pcl::fromROSMsg(*cloud_msg, extract_info->cloud);
            _fill_info(extract_info->cloud, point_info, extract_info->segments);
            _extract_features(point_info, extract_info->segments, extract_info->label);
            recycled.push_back(extract_info);
            if(recycled.size() > 32) recycled.pop_front();
            _m_cloud_extract_buf.lock();
            _cloud_extract_buf.push_back(extract_info);
            _m_cloud_extract_buf.unlock();

            int drop_frame_count = drop_with_lock(_cloud_msg_buf, _m_cloud_msg_buf, 4, 0);
            if(drop_frame_count > 0)
            {
                ROS_WARN("pop %d old cloud msg for real_time performance", drop_frame_count);
            }
        }
    }

    void _deskew_process()
    {
        ros::NodeHandle nh;
        ros::Rate rate_100(100);
        ros::Time t_o0, t_base, t_p0, t_pn, fixed_stamp;
        CloudExtractInfo::Ptr extract_cloud_info;

        while (nh.ok())
        {
            if(_cloud_extract_buf.empty())
            {
                rate_100.sleep();
                continue;
            }

            // wait for first odom and stamp t_o0
            wait_and_pop_with_lock(_odom_msg_buf, _m_odom_msg_buf, ros::Time(0), 100, t_o0, nh);

            wait_and_get_with_lock(_cloud_extract_buf, _m_cloud_extract_buf, t_o0, 100, extract_cloud_info, nh);

            fixed_stamp = ROS_TIME(extract_cloud_info);
            CloudType &cloud_full = extract_cloud_info->cloud;
            auto& segment = extract_cloud_info->segments;

            t_p0 = fixed_stamp;
            float max_rel_time = 0;
            for(const auto& seg : segment)
            {
                max_rel_time = std::max(max_rel_time, cloud_full[seg.second-1].rel_time);
            }
            t_pn = fixed_stamp + ros::Duration(max_rel_time);

            // stamp of last odom must be later than last point
            while (ENABLE_DESKEW && nh.ok())
            {
                {
                    std::lock_guard<std::mutex> lock_guard(_m_odom_msg_buf);
                    if (t_pn < ROS_TIME(_odom_msg_buf.back()))
                        break;
                }
                // ROS_INFO("wait odom");
                rate_100.sleep();
            }

            Eigen::Isometry3d P0_inv, Pi;
            _inter_next_odom(t_p0, extract_cloud_info->pose);
            extract_cloud_info->pose = extract_cloud_info->pose * T_ODOMETER_LIDAR;
            P0_inv = extract_cloud_info->pose.inverse();

            // deskewing
            if (ENABLE_DESKEW)
            {
                std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<std::pair<float, int>>> prior_q;
                for(int i = 0; i < segment.size(); ++i)
                {
                    prior_q.emplace(cloud_full[segment[i].first].rel_time, i);
                }
                int top;
                Eigen::Vector3d new_p;
                ros::Duration dt;
                Eigen::Vector3d old_p;
                while(!prior_q.empty())
                {
                    top = prior_q.top().second;
                    prior_q.pop();

                    auto& p = cloud_full[segment[top].first++];
                    _inter_next_odom(fixed_stamp + dt.fromSec(p.rel_time), Pi);
                    new_p = P0_inv * Pi * T_ODOMETER_LIDAR * (old_p << p.x, p.y, p.z).finished();
                    p.x = new_p[0];
                    p.y = new_p[1];
                    p.z = new_p[2];
                    
                    if(segment[top].first < segment[top].second)
                    {
                        prior_q.emplace(cloud_full[segment[top].first].rel_time, top);
                    }
                }
            }

            _publish(*extract_cloud_info);

            int drop_frame_count = drop_with_lock(_cloud_extract_buf, _m_cloud_extract_buf, 4, 0);
            if(drop_frame_count > 0)
            {
                ROS_WARN("pop %d old cloud extraction info(s) for real_time performance", drop_frame_count);
            }
        }
    }

    void _fill_info(const CloudType& cloud, std::vector<PointInfo>& point_info, std::vector<std::pair<int, int>>& segment) const
    {
        const auto size = cloud.size();
        point_info.resize(size);
        segment.clear();
        int last_seg_ind = 0;
        for(int i = 1; i < size; ++i)
        {
            point_info[i].range = _calc_dist(cloud[i]);
            if(cloud[i-1].ring != cloud[i].ring)
            {
                segment.emplace_back(last_seg_ind, i);
                last_seg_ind = i;
                point_info[i].angle_diff = 0;
                continue;
            }
            point_info[i].angle_diff = _calc_angel(cloud[i-1], cloud[i]);
        }
        segment.emplace_back(last_seg_ind, size);
    }

    void _publish(CloudExtractInfo &info)
    {
        static CloudType cloud_sharp, cloud_flat;
        static geometry_msgs::PoseStamped odom_lidar;
        static sensor_msgs::PointCloud2 cloud_sharp_msg, cloud_flat_msg;
        static wloam::CloudInfo cloud_info_msg;

        cloud_sharp.clear();
        cloud_flat.clear();
        const auto size = info.label.size();
        for(int i = 0; i < size; ++i)
        {
            if(info.label[i] == LABEL_SHARP)
                cloud_sharp.push_back(info.cloud[i]);
            else if(info.label[i] == LABEL_FLAT)
                cloud_flat.push_back(info.cloud[i]);
        }

        removeClosedPointCloud(info.cloud, info.cloud, MIN_POINT_DIST);
        removeClosedPointCloud(cloud_sharp, cloud_sharp, MIN_POINT_DIST);
        removeClosedPointCloud(cloud_flat, cloud_flat, MIN_POINT_DIST);

        pcl::toROSMsg(cloud_sharp, cloud_sharp_msg);
        cloud_sharp_msg.header = info.header;
        _pub_feature_sharp.publish(cloud_sharp_msg);

        pcl::toROSMsg(cloud_flat, cloud_flat_msg);
        cloud_flat_msg.header = info.header;
        _pub_feature_flat.publish(cloud_flat_msg);

        Eigen::Vector3d t(info.pose.translation());
        Eigen::Quaterniond q(info.pose.linear());
        odom_lidar.header = info.header;
        odom_lidar.pose.position.x = t.x();
        odom_lidar.pose.position.y = t.y();
        odom_lidar.pose.position.z = t.z();
        odom_lidar.pose.orientation.w = q.w();
        odom_lidar.pose.orientation.x = q.x();
        odom_lidar.pose.orientation.y = q.y();
        odom_lidar.pose.orientation.z = q.z();

        _pub_odom_lidar.publish(odom_lidar);

        cloud_info_msg.Odometry = odom_lidar.pose;
        cloud_info_msg.PointsSharp = cloud_sharp_msg;
        cloud_info_msg.PointsFlat = cloud_flat_msg;
        pcl::toROSMsg(info.cloud, cloud_info_msg.PointsFull);
        cloud_info_msg.PointsFull.header = info.header;

        _pub_cloud_info.publish(cloud_info_msg);

        _pub_cloud_deskewed.publish(cloud_info_msg.PointsFull);
    }

    void _extract_features(const std::vector<PointInfo>& point_info, const std::vector<std::pair<int,int>>& segment, std::vector<char> &label) const
    {
        static std::vector<std::pair<int, int>> cut_point;
        label.resize(point_info.size(), 0);
        for (const auto& seg : segment)
        {
            cut_point.resize(0);
            int last_cut_point = seg.first;
            for (int j = seg.first + 1; j < seg.second; ++j)
            {
                const float range_diff = point_info[j].range - point_info[j-1].range;
                const bool over_angle_thres = point_info[j].angle_diff > ANGLE_DIFF_THREHOLD;

                // if(over_angle_thres)
                // {
                //     std::cerr << "over angle " << point_info[j].angle_diff / M_PI * 180.0 << "\n" << point_info[j].range << std::endl;           
                // }

                if (!over_angle_thres && std::abs(range_diff) < DISTANCE_DIFF_THRESHOLD)
                    continue;

                cut_point.emplace_back(last_cut_point, j);
                

                last_cut_point = j;
            }
            cut_point.emplace_back(last_cut_point, seg.second);

            for(int i = 1; i < cut_point.size(); ++i)
            {
                const auto& left = cut_point[i].first;
                const auto& right = cut_point[i].second;
                const auto diff = point_info[left].range - point_info[left-1].range;
                // if(right - left < MIN_SEG_LEN || cut_point[i-1].second - cut_point[i-1].first < MIN_SEG_LEN)
                //     continue;
                if(point_info[left].angle_diff > ANGLE_DIFF_THREHOLD)
                    label[left] = label[left-1] = LABEL_SHARP;
                else
                    label[diff < 0 ? left : left - 1] = LABEL_SHARP;
            }

            for (const auto piece : cut_point)
            {
                _label_segment(point_info, piece.first, piece.second, label);
            }
        }
    }

    float _calc_diff(const float range1, const float range2, const float angle) const
    {
        return atan((range2 - range1) / (range1 * angle));
    }

    float _calc_angel(const CloudType::PointType& p1, const CloudType::PointType& p2) const
    {
        return std::acos(Eigen::Vector3f(p1.x, p1.y, p1.z).normalized().dot(Eigen::Vector3f(p2.x, p2.y, p2.z).normalized()));
    }

    float _calc_dist(const CloudType::PointType& p) const
    {
        return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    void _label_segment(const std::vector<PointInfo>& point_info, const int begin_ind, const int end_ind, std::vector<char>& label) const
    {
        int seg_len = end_ind - begin_ind;
        if (seg_len < MIN_SEG_LEN)
            return;
        
        const int radius = std::min(NEIGHBOR_RADIUS, seg_len / 2);
        using IndexedSmoothness = std::pair<int, float>;
        static std::vector<IndexedSmoothness> smoothness;
        static std::vector<int> picked;

        smoothness.resize(0);
        picked.resize(seg_len, 0);

        const int start = begin_ind + radius;
        const int end = end_ind - radius;
        for (int ind = start; ind < end; ++ind)
        {
            const float cur_depth = point_info[ind].range;

            float diff = 0;
            float angle1 = 0;
            float angle2 = 0;
            
            for (int i = 1; i <= radius; ++i)
            {
                angle1 += point_info[ind+i].angle_diff;
                angle2 += point_info[ind-i+1].angle_diff;

                float diff1 = _calc_diff(cur_depth,  point_info[ind+i].range, angle1);
                float diff2 = _calc_diff(cur_depth, point_info[ind-i+1].range, angle2);
                diff += (diff1 + diff2);
            }

            diff = std::abs(diff) / radius;
            smoothness.emplace_back(ind, diff);
        }

        std::sort(smoothness.begin(), smoothness.end(), [](const IndexedSmoothness &u, const IndexedSmoothness &v) {
            return u.second < v.second;
        });

        auto rp = smoothness.rbegin();
        while (rp != smoothness.rend())
        {
            if (rp->second < SHARP_THRESHOLD)
                break;
            const int ind = rp->first;
            const int picked_ind = rp->first - begin_ind;
            if (picked[picked_ind] == 0)
            {
                label[ind] = LABEL_SHARP;
                for (int i = 1; i <= radius; ++i)
                {
                    picked[picked_ind + i] = 1;
                    picked[picked_ind - i] = 1;
                }
            }
            ++rp;
        }

        auto p = smoothness.begin();
        while (p != smoothness.end())
        {
            if (p->second > SMOOTH_THRESHOLD)
                break;
            const int ind = p->first;
            const int picked_ind = p->first - begin_ind;
            if (picked[picked_ind] == 0)
            {
                label[ind] = LABEL_FLAT;
                // for(int i = 1; i <= radius; ++i)
                // {
                //     picked[picked_ind + i] = 1;
                //     picked[picked_ind - i] = 1;
                // }
            }
            ++p;
        }
    }

    void _inter_next_odom(const ros::Time &stamp, Eigen::Isometry3d &odom)
    {      
        static geometry_msgs::PoseStampedConstPtr odom_msg_pre, odom_msg_next;
        static double t_raw[3], q_pre_raw[4], q_next_raw[4];
        static Eigen::Map<Eigen::Vector3d> t(t_raw);
        static Eigen::Map<Eigen::Quaterniond> q_pre(q_pre_raw), q_next(q_next_raw);
        if(odom_msg_next == nullptr || !(ROS_TIME(odom_msg_pre) <= stamp && stamp < ROS_TIME(odom_msg_next)))
        {
            std::lock_guard<std::mutex> lock_guard(_m_odom_msg_buf);
            auto p_next = std::upper_bound(_odom_msg_buf.begin(), _odom_msg_buf.end(), stamp, [](const ros::Time &t, const geometry_msgs::PoseStampedConstPtr &element) {
                return t < ROS_TIME(element);
            });
            _odom_msg_buf.erase(_odom_msg_buf.begin(), p_next - 1);

            odom_msg_pre = _odom_msg_buf[0];
            odom_msg_next = _odom_msg_buf[1];
        }

        double so = (stamp - ROS_TIME(odom_msg_pre)).toSec() / (ROS_TIME(odom_msg_next) - ROS_TIME(odom_msg_pre)).toSec();
        t_raw[0] = odom_msg_pre->pose.position.x + so * (odom_msg_next->pose.position.x - odom_msg_pre->pose.position.x);
        t_raw[1] = odom_msg_pre->pose.position.y + so * (odom_msg_next->pose.position.y - odom_msg_pre->pose.position.y);
        t_raw[2] = odom_msg_pre->pose.position.z + so * (odom_msg_next->pose.position.z - odom_msg_pre->pose.position.z);

        q_pre_raw[0] = odom_msg_pre->pose.orientation.x;
        q_pre_raw[1] = odom_msg_pre->pose.orientation.y;
        q_pre_raw[2] = odom_msg_pre->pose.orientation.z;
        q_pre_raw[3] = odom_msg_pre->pose.orientation.w;

        q_next_raw[0] = odom_msg_next->pose.orientation.x;
        q_next_raw[1] = odom_msg_next->pose.orientation.y;
        q_next_raw[2] = odom_msg_next->pose.orientation.z;
        q_next_raw[3] = odom_msg_next->pose.orientation.w;

        odom.setIdentity();
        odom.rotate(q_pre.slerp(so,q_next));
        odom.pretranslate(t);
    }

    void _cloud_msg_handler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock_guard(_m_cloud_msg_buf);
        _cloud_msg_buf.push_back(msg);
    }

    void _odom_msg_handler(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock_guard(_m_odom_msg_buf);
        _odom_msg_buf.push_back(msg);
    }

    static constexpr char LABEL_SHARP = 255;
    static constexpr char LABEL_FLAT = 128;    
    double MIN_POINT_DIST;
    float SMOOTH_THRESHOLD;
    float SHARP_THRESHOLD;
    int NEIGHBOR_RADIUS;
    int MIN_SEG_LEN;
    float ANGLE_DIFF_THREHOLD;
    float DISTANCE_DIFF_THRESHOLD;
    bool ENABLE_DESKEW = true;
    Eigen::Isometry3d T_ODOMETER_LIDAR;

    std::mutex _m_cloud_msg_buf, _m_odom_msg_buf, _m_cloud_extract_buf;
    std::deque<sensor_msgs::PointCloud2ConstPtr> _cloud_msg_buf;
    std::deque<geometry_msgs::PoseStampedConstPtr> _odom_msg_buf;
    std::deque<CloudExtractInfo::Ptr> _cloud_extract_buf;

    std::thread _extraction_thread, _deskew_thread;
    
    ros::Publisher _pub_feature_flat, _pub_feature_sharp, _pub_cloud_deskewed, _pub_cloud_info, _pub_odom_lidar;

    ros::Subscriber _sub_cloud_msg, _sub_odom_msg;

}; // class ScanExtraction

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanExtraction");
    ros::NodeHandle nh;

    ScanExtraction ext;
    ext.SpinOnMultiQueue();
}