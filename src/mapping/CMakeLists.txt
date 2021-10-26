add_executable(scanExtraction scanExtraction.cpp)
target_link_libraries(scanExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(scanExtraction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(ApaMapping ApaMapping.cpp)
target_link_libraries(ApaMapping gtsam -llz4 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ApaMapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
