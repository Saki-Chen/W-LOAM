# W-LOAM
A ros package for robust odometry and mapping using LiDAR with aid of different sensors

## Demo Video
https://www.bilibili.com/video/BV1Fy4y1L7kZ?from=search&seid=6228704901331074158&spm_id_from=333.337.0.0

## Sensor Configure
1. wheel encoder + steer encoder + LiDAR   
   This is proposed by this repo   
2. IMU + LiDAR   
   The imuOdometry is implemented based on  
    https://github.com/TixiaoShan/LIO-SAM/blob/master/src/imuPreintegration.cpp
3. configure 1 or 2 + GPS   
   For mapping a large area, GPS is favored    

## Dependency
* ROS (tested with melodic)   
  ```bash
  sudo apt install ros-melodic-gps-common
  # optional
  # if you want to visualize with satellite maps, use rviz plugin rviz_satellite
  cd YOUR_CATKIN_WS/src
  git clone https://github.com/Saki-Chen/rviz_satellite
  cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release
  ```
* gtsam (tested with 4.0.3)   
  refer to https://github.com/borglab/gtsam   
  or for ubuntu 18.04
  ```bash
  git clone https://github.com/borglab/gtsam
  cd gtsam
  git checkout 4.0.3
  mkdir build && cd build
  cmake ..
  make -j8
  sudo make install
  ```
## Build
```bash
cd YOUR_CATKIN_WS/src
git clone https://github.com/Saki-Chen/apa_msgs
git clone https://github.com/Saki-Chen/W-LOAM
cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release
```
## Data Format
1. Point
``` 
    float32 x   
    float32 y      
    float32 z    
    float32 intensity
    // id of the laser scaner
    uint16 ring
    // time relative to the header stamp  
    float32 rel_time    
```
2. Wheel Counting
```
    std_msgs/Header header
    // four counter for four wheel
    int64 FL
    int64 FR
    int64 RL
    int64 RR
```

3. Steer Angle
```
    std_msgs/Header header
    // positive for clock-wise
    float64 angle
```

4. IMU (sensor_msgs/Imu )

## Calibration
The extrinsic for sensors is expressed as urdf file in folder launch/config.    
Especially, the parameter file for Wheel Odometry is in vehicle_params. 
## Run
1. using provided test bag
```
1. Download bag
https://pan.baidu.com/s/1v_jl-j4jdTZGjtW3P6c7Eg   
password: nk5a

2. run algorithm
// with wheel odometry
roslaunch wloam run.launch simulation:=true
// or with imu odometry
roslaunch wloam run.launch simulation:=true odometry_link:=imu_link

3. run rviz
roslaunch wloam rviz.launch

4. play bag
rosbag play parking-lot.bag --clock
```
2. using dataset provided by lio-sam
```
goto https://github.com/TixiaoShan/LIO-SAM
1. Find and Download bag
Walking, Park and Garden is tested

2. run algorithm
roslaunch wloam run.launch laser_topic:=points_raw imu_topic:=imu_raw cloud_format:=velodyne robot_name:=lio odometry_link:=imu_link simulation:=true

3. run rviz
roslaunch wloam rviz.launch

4. play bag
rosbag play park.bag --clock

If you want to test GPS, just add option enable_gps:=true when start launch file and check the AerialMapDisplay in rviz.
```

3. using Livox data
```
goto https://github.com/KIT-ISAS/lili-om
1. Find and Download bag
using KA_Urban_Schloss_1.bag as example

2. run algorithm
roslaunch wloam run.launch laser_topic:=points_raw imu_topic:=imu/data cloud_format:=velodyne robot_name:=livox odometry_link:=imu_link simulation:=true

3. run convertion for livox data
rosrun wloam livox_converter

4. run rviz
roslaunch wloam rviz.launch

5. play bag
rosbag play KA_Urban_Schloss_1.bag --clock imu/data:=nouse gnss:=gps/fix

If you want to test GPS, just add option enable_gps:=true when start launch file and check the AerialMapDisplay in rviz.
```

## Paper
```
WLOAM Wheel-LiDAR Odometry and Mapping for Autonomous Vehicles
comming soon...
```

Imu Odometry is adapted from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
```
@inproceedings{liosam2020shan,
  title={LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping},
  author={Shan, Tixiao and Englot, Brendan and Meyers, Drew and Wang, Wei and Ratti, Carlo and Rus Daniela},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={5135-5142},
  year={2020},
  organization={IEEE}
}
```
## Acknowledgement
This work is inspired by LOAM(Lidar Odometry and Mapping in Real-time) and LIO-SAM(Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping)