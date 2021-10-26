#pragma once

#include <ros/time.h>
#include <mutex>

template <typename T>
const ros::Time& ROS_TIME(const T& msg)
{
    return msg->header.stamp;
}

template <typename T1, typename T2>
bool lower(T1 msg1, T2 msg2)
{
    return ROS_TIME(msg1) < ROS_TIME(msg2);
}

template <typename T1, typename T2>
void  wait_and_get_with_lock(T1& buf, T2& m_buf, const ros::Time& stamp, const double freq, typename T1::value_type& front, const ros::NodeHandle& nh = ros::NodeHandle())
{
    ros::Rate rate(freq);
    while(nh.ok())
    {
        {
            std::lock_guard<T2> lock_guard(m_buf);
            if(!buf.empty() && ROS_TIME(buf.back()) > stamp)
            {
                while (ROS_TIME(buf.front()) <= stamp)
                    buf.pop_front();
                front.swap(buf.front());
                buf.pop_front();
                return;
            }
        }
        rate.sleep();
    }
}

template<typename T>
void wait_and_pop(T& buf, const ros::Time& stamp, const double freq, ros::Time& next_stamp, const ros::NodeHandle& nh = ros::NodeHandle())
{
    ros::Rate rate(freq);
    next_stamp = stamp;
    while(nh.ok())
    {
        if(!buf.empty() && ROS_TIME(buf.back()) > stamp)
        {
            while (ROS_TIME(buf.front()) <= stamp)
                buf.pop_front();
            next_stamp = ROS_TIME(buf.front());
            return;
        }
        rate.sleep();
    }
}

template<typename T1, typename T2>
void wait_and_pop_with_lock(T1& buf, T2& m_buf, const ros::Time& stamp, const double freq, ros::Time& next_stamp, const ros::NodeHandle& nh = ros::NodeHandle())
{
    ros::Rate rate(freq);
    next_stamp = stamp;
    while(nh.ok())
    {
        {
            std::lock_guard<T2> lock_guard(m_buf);
            if(!buf.empty() && ROS_TIME(buf.back()) > stamp)
            {
                while (ROS_TIME(buf.front()) <= stamp)
                    buf.pop_front();
                next_stamp = ROS_TIME(buf.front());
                return;
            }
        }
        rate.sleep();
    }
}

template<typename T1, typename T2>
int drop_with_lock(T1& buf, T2& m_buf, const int buf_seg_length, const int reserve_size)
{
    std::lock_guard<T2> lock_guard(m_buf);
    if(buf.size() <= reserve_size) return 0;
    const int drop_frame_count = (buf.size() - reserve_size) / buf_seg_length * (buf_seg_length / 2);
    for(int i = 0; i < drop_frame_count; ++i)
    {
        buf.pop_front();
    }
    return drop_frame_count;
}
