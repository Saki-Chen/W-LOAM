add_executable(DeadReckoning DeadReckoning.cpp wheel_odometry.cpp)
target_link_libraries(DeadReckoning -ljsoncpp ${catkin_LIBRARIES})
add_dependencies(DeadReckoning ${catkin_EXPORTED_TARGETS})

add_executable(imuOdometry_fix_lag imuOdometry_fix_lag.cpp)
target_link_libraries(imuOdometry_fix_lag gtsam gtsam_unstable ${catkin_LIBRARIES})
add_dependencies(DeadReckoning ${catkin_EXPORTED_TARGETS})
