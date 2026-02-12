#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

// C++
#include <iostream>
#include <vector>
#include <cmath>


// ACADOS
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// Generated C code
#include "haloball_model/haloball_model.h"
#include "acados_solver_haloball.h"

using namespace std;
using std::vector;

struct OptTraj {
    bool   is_solved;
    double x[HALOBALL_N+1];
    double y[HALOBALL_N+1];
    double yaw[HALOBALL_N+1];
    double time_solving;
    double cost;
    double right_wheel_angular_vel[HALOBALL_N], left_wheel_angular_vel[HALOBALL_N];
};

struct X0 {
    double x;
    double y;
    double yaw;
};

struct Yref {
    double x[HALOBALL_N];
    double y[HALOBALL_N];
    double yaw[HALOBALL_N];
    double right_wheel_angular_vel[HALOBALL_N], left_wheel_angular_vel[HALOBALL_N];
};

struct Yref_e {
    double x;
    double y;
    double yaw;
};