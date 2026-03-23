// mpc_controller.hpp

#pragma once

// ROS
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
#include <array>
#include <iomanip>
#include <limits>

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

class MPCController : public rclcpp::Node {
public:
    MPCController();
    ~MPCController();

    static constexpr int control_frequency = 10; // Hz

    // Main control loop
    void control_loop();

private:
    // Solver parameters
    static constexpr int N_Horizon = 25; // Number of intervals in the horizon
    static constexpr double T_Horizon = 5; // (s) predict horizon   
    static constexpr double dt = T_Horizon / N_Horizon; // (s) sampling time
    static constexpr int NX = HALOBALL_NX;  // state dimension
    static constexpr int NU = HALOBALL_NU;  // control dimension
    static constexpr int NY = HALOBALL_NY;  // output dimension for cost
    static constexpr int NYN = HALOBALL_NYN;  // output dimension for terminal cost

    static constexpr double goal_tolerance = 0.05;    // For control loop

    // ---------- Variables Declaration ----------
    // ROS Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;

    // ROS Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;

    // State variables
    std::array<double, NX> x0{}; // current state x0 = [x, y, theta]
    std::array<double, NU> u{}; // control state u = [linear_vel, angular_vel]
    std::array<double, (N_Horizon + 1) * NX> x_ref{}; // state reference
    std::array<double, N_Horizon * NU> u_ref{}; // control reference

    // unwrap_odom_yaw variables
    double prev_yaw = 0.0, unwrapped_yaw = 0.0;
    bool first_odom = true;

    // ROS parameters
    std::string frame_id;
    double weight_x, weight_y, weight_yaw, weight_linear_vel, weight_angular_vel;

    // Global path callback variables
    nav_msgs::msg::Path global_path;
    size_t global_path_index = 0;

    // Flags
    bool solver_initialized = false, odom_received = false, goal_reached = false;

    // ACADOS solver handles
    haloball_solver_capsule* acados_ocp_capsule;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_solver *nlp_solver;
    // -------- Variables Declaration Ends --------

    // Callbacks (ConstPtr& - Read Only)
    void odom_callback(const nav_msgs::msg::Odometry::ConstPtr& msg);
    void global_path_callback(const nav_msgs::msg::Path::ConstPtr& msg);

    // Functions
    void initialize_solver();
    void set_weights();
    void update_global_path_index();
    void compute_reference();
    void set_stage_cost_reference();
    void print_solution_info();
    void publish_control();
    void publish_predicted_path();
    double unwrap_yaw(double current_odom_yaw);
    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);    
};