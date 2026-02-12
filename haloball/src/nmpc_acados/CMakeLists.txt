cmake_minimum_required(VERSION 3.8)
project(nmpc_acados)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

############################################################################
set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)
# find dependencies
find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS dependencies)
  find_package(${pkg} REQUIRED)
endforeach()

############################################################################
# $ENV{HOME} - home directory
set(acados_include "$ENV{HOME}/acados/include")
set(acados_lib "$ENV{HOME}/acados/lib")
set(full_model_build "${PROJECT_SOURCE_DIR}/scripts/c_generated_code")

include_directories(
  include
  ${acados_include}
  ${acados_include}/blasfeo/include/
  ${acados_include}/hpipm/include/
  ${acados_include}/acados/
  ${acados_include}/qpOASES_e/
  ${full_model_build}
)

link_directories(${acados_lib})
link_directories(${full_model_build})

############################################################################
add_executable(nmpc_acados_node src/nmpc_acados_node.cpp)
ament_target_dependencies(nmpc_acados_node 
  ${dependencies}
)
target_link_libraries(nmpc_acados_node
  ${full_model_build}/libacados_ocp_solver_haloball.so
  ${acados_lib}/libacados.so
)
add_executable(odom_path_publisher src/odom_path_publisher.cpp)
ament_target_dependencies(odom_path_publisher
  ${dependencies}
)
add_executable(trajectory_publisher_node src/trajectory_publisher.cpp)
ament_target_dependencies(trajectory_publisher_node 
  ${dependencies}
)

############################################################################
install(
  TARGETS
  nmpc_acados_node
  odom_path_publisher
  trajectory_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
  launch 
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
  ${full_model_build}/libacados_ocp_solver_haloball.so
  DESTINATION lib
)

############################################################################
ament_package()
