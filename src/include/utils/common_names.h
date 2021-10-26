#pragma once
#include <string>
#include "utils/frame_id.h"

static constexpr char topic_wheeling_count[]("wheeling_count_fl_fr_rl_rr");
static constexpr char topic_steering_angle[]("steering_angle_deg");
static constexpr char topic_chasis_speed[]("chasis_speed");
static constexpr char topic_chasis_pedal_status[]("chasis_pedal_status");
static constexpr char topic_odometer[]("odometer");
static constexpr char topic_velometer[]("velometer");
static constexpr char topic_extra_odom_info[]("extra_odom_info");
static constexpr char topic_vhicle_pose2D[]("vehicle_pose2D");
static constexpr char topic_tracked_vehicle[]("tracked_vehicle");
static constexpr char topic_slot_info[]("parking_slot_info");
static constexpr char topic_laser_cloud_raw[]("points_raw");
static constexpr char topic_laser_cloud_full[]("laser_cloud_full");
static constexpr char topic_laser_cloud_sharp[]("laser_cloud_corner");
static constexpr char topic_laser_cloud_flat[]("laser_cloud_surf");
static constexpr char topic_laser_cloud_registered[]("laser_cloud_registered");
static constexpr char topic_laser_cloud_surround[]("laser_cloud_local_full_map");
static constexpr char topic_laser_cloud_surround_corner[]("laser_cloud_local_corner_map");
static constexpr char topic_laser_cloud_surround_surf[]("laser_cloud_local_surf_map");
static constexpr char topic_laser_cloud_map[]("laser_cloud_global_map");
static constexpr char topic_laser_cloud_map_corner[]("laser_cloud_global_corner_map");
static constexpr char topic_laser_cloud_map_surf[]("laser_cloud_global_surf_map");
static constexpr char topic_odometry_lidar[]("odometry_lidar");
static constexpr char topic_odometry_lidar_optimized[]("odometry_lidar_optimized");
static constexpr char topic_cloud_deskewed_info[]("cloud_deskewed_info");
static constexpr char topic_cloud_info[]("cloud_info");
static constexpr char topic_gps_odom[]("gps/odom");
static constexpr char topic_gps_fix[]("gps/fix");
static constexpr char topic_imu_msg[]("imu/data");