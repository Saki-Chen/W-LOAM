add_executable(odometry_publisher odometry_publisher.cpp)
target_link_libraries(odometry_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(odometry_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(visualization visualization.cpp)
target_link_libraries(visualization -ljsoncpp ${catkin_LIBRARIES})
add_dependencies(visualization ${catkin_EXPORTED_TARGETS})

add_executable(gps_odom_publisher gps_odom_publisher.cpp)
target_link_libraries(gps_odom_publisher ${catkin_LIBRARIES})
add_dependencies(gps_odom_publisher ${catkin_EXPORTED_TARGETS})

add_executable(evaluation evaluation.cpp)
target_link_libraries(evaluation ${catkin_LIBRARIES})
add_dependencies(evaluation ${catkin_EXPORTED_TARGETS})

add_executable(pcd_to_topic pcd_to_topic.cpp)
target_link_libraries(pcd_to_topic ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pcd_to_topic ${catkin_EXPORTED_TARGETS})
