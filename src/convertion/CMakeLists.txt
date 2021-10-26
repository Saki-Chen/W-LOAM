add_executable(points_converter points_converter.cpp)
target_link_libraries(points_converter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(points_converter ${catkin_EXPORTED_TARGETS})

add_executable(livox_converter livox_converter.cpp)
target_link_libraries(livox_converter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(livox_converter ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})