cmake_minimum_required(VERSION 3.8) # min cmake version
project(haloball_bringup)                    # package name

# Compiler Options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Create cpp executable and link with dependencies
add_executable(base_link_tf src/base_link_tf.cpp)
ament_target_dependencies(base_link_tf
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

# Install cpp executables
install(TARGETS 
  base_link_tf
  DESTINATION lib/${PROJECT_NAME}
)

# Install files
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
