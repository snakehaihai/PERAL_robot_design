cmake_minimum_required(VERSION 3.8) # min cmake version
project(haloball_bringup)                    # package name

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  laser_geometry
)

# Find ROS dependencies
find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS dependencies)
  find_package(${pkg} REQUIRED)
endforeach()
# Find C++ dependencies
find_package(PCL REQUIRED)

include_directories(
  include # cpp "include" directory
  ${PCL_INCLUDE_DIRS}
)

# Create cpp executable and link with dependencies
add_executable(base_link_tf src/base_link_tf.cpp)
ament_target_dependencies(base_link_tf
  rclcpp nav_msgs geometry_msgs tf2_ros
)

add_executable(pointcloud_filter src/pointcloud_filter.cpp)
ament_target_dependencies(pointcloud_filter 
  rclcpp sensor_msgs pcl_conversions
)
target_link_libraries(pointcloud_filter ${PCL_LIBRARIES})

add_executable(laser_to_pointcloud src/laser_to_pointcloud.cpp)
ament_target_dependencies(laser_to_pointcloud 
  rclcpp sensor_msgs laser_geometry tf2_ros
)
# For testing purpose
add_executable(odom_callback src/odom_callback.cpp)
ament_target_dependencies(odom_callback
  rclcpp nav_msgs tf2_ros tf2_geometry_msgs
)

# Install cpp executables
install(TARGETS 
  base_link_tf
  pointcloud_filter
  laser_to_pointcloud
  odom_callback # For testing purpose
  DESTINATION lib/${PROJECT_NAME}
)

# Install python scripts
install(PROGRAMS
  scripts/odometry_simulator.py
  scripts/marker.py
  scripts/nlp_to_nav2.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directory
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
