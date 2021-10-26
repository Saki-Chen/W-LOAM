#pragma once

#include <string>
#include <iostream>
#include <tf/transform_listener.h>

tf::StampedTransform read_static_tf(const std::string& target_frame, const std::string& source_frame, double time_out = 3)
{
    static tf::TransformListener listener;
    ros::Time::waitForValid();
    tf::StampedTransform T;
    T.setIdentity();
    try
    {
        listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(time_out));
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), T);

    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Time OUT! Waiting %f second(s) for static transform from %s to %s, returning Identity", time_out, source_frame.c_str(), target_frame.c_str());
    }
        
    return T;
}
