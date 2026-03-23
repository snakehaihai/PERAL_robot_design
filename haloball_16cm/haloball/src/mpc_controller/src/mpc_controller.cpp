// mpc_controller.cpp

#include "mpc_controller.hpp"

MPCController::MPCController() : Node("mpc_controller") {  
    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&MPCController::odom_callback, this, std::placeholders::_1));

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/global_path", 1,
        std::bind(&MPCController::global_path_callback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    predicted_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/predicted_path", 10);
    
    // Declare and get ROS parameters
    frame_id = this->declare_parameter("frame_id", "odom");
    weight_x = this->declare_parameter("weight_x", 5.0);
    weight_y = this->declare_parameter("weight_y", 5.0);
    weight_yaw = this->declare_parameter("weight_yaw", 0.75);
    weight_linear_vel = this->declare_parameter("weight_linear_vel", 100.0);
    weight_angular_vel = this->declare_parameter("weight_angular_vel", 0.2);

    std::cout << "===== Parameter Values =====" << std::endl;
    std::cout << "weight_x: " << weight_x << std::endl;
    std::cout << "weight_y: " << weight_y << std::endl;
    std::cout << "weight_yaw: " << weight_yaw << std::endl;
    std::cout << "weight_linear_vel: " << weight_linear_vel << std::endl;
    std::cout << "weight_angular_vel: " << weight_angular_vel << std::endl;
    std::cout << "===========================" << std::endl;

    initialize_solver();
}

// Custom C++ destructor just for safety. It already have auto destructor.
MPCController::~MPCController() {
    // Clean up solver
    if (acados_ocp_capsule != nullptr) {
        haloball_acados_free(acados_ocp_capsule);
        haloball_acados_free_capsule(acados_ocp_capsule);
    }
}

void MPCController::odom_callback(const nav_msgs::msg::Odometry::ConstPtr& msg) {
    x0[0] = msg->pose.pose.position.x;
    x0[1] = msg->pose.pose.position.y;
    double odom_yaw = tf2::getYaw(msg->pose.pose.orientation);  // [-pi, pi]
    x0[2] = unwrap_yaw(odom_yaw);
    
    odom_received = true;
    // Debug
    // std::cout << "x = " << x0[0] << ", y = " << x0[1] << ", yaw_unwrapped = " << x0[2] << std::endl;
}

void MPCController::global_path_callback(const nav_msgs::msg::Path::ConstPtr& msg) {
    global_path = *msg;
    // Debug
    // std::cout << "Received new trajectory with " << msg->poses.size() << " poses." << std::endl;
}

void MPCController::initialize_solver() {
    if (solver_initialized) {
        return;
    }
    
    // Create solver capsule
    acados_ocp_capsule = haloball_acados_create_capsule();
    // Create solver
    int status = haloball_acados_create(acados_ocp_capsule);
    if (status) {
        std::cerr << "Failed to create acados solver with status: " << status << std::endl;
        return;
    }
    
    nlp_config = haloball_acados_get_nlp_config(acados_ocp_capsule);
    nlp_dims = haloball_acados_get_nlp_dims(acados_ocp_capsule);
    nlp_in = haloball_acados_get_nlp_in(acados_ocp_capsule);
    nlp_out = haloball_acados_get_nlp_out(acados_ocp_capsule);
    nlp_solver = haloball_acados_get_nlp_solver(acados_ocp_capsule);

    solver_initialized = true;
    std::cout << "Solver initialized successfully!" << std::endl;
}
void MPCController::set_weights() {
    double W[NY * NY] = {0};
    double W_e[NYN * NYN] = {0};

    // Stage cost weight
    W[0 * NY + 0] = weight_x;
    W[1 * NY + 1] = weight_y; 
    W[2 * NY + 2] = weight_yaw;
    W[3 * NY + 3] = weight_linear_vel;
    W[4 * NY + 4] = weight_angular_vel;

    // Terminal stage cost weight
    W_e[0 * NYN + 0] = weight_x;
    W_e[1 * NYN + 1] = weight_y;
    W_e[2 * NYN + 2] = weight_yaw;

    for (int i = 0; i < N_Horizon; i++) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N_Horizon, "W", W_e);
}

void MPCController::update_global_path_index() {
    // Advance global_path_index as long as the robot has longitudinally passed the current waypoint
    const size_t path_size = global_path.poses.size();

    while (global_path_index + 1 < path_size) {
        const double wp_x   = global_path.poses[global_path_index].pose.position.x;
        const double wp_y   = global_path.poses[global_path_index].pose.position.y;
        const double next_x = global_path.poses[global_path_index + 1].pose.position.x;
        const double next_y = global_path.poses[global_path_index + 1].pose.position.y;

        /* Dot product of (robot - waypoint) and (next_waypoint - waypoint)
        Positive means the robot is ahead of current waypoint along the path direction */
        const double dot = (x0[0] - wp_x) * (next_x - wp_x) + (x0[1] - wp_y) * (next_y - wp_y);
        if (dot > 0.0) global_path_index++;
        else break;
    }
}

void MPCController::compute_reference() {
    const size_t path_size = global_path.poses.size();
    
    // Compute x_ref
    for (int i = 0; i <= N_Horizon; i++) {
        const int idx = (global_path_index + i < path_size) ? (global_path_index + i) : (path_size - 1);
        x_ref[i * NX + 0] = global_path.poses[idx].pose.position.x;
        x_ref[i * NX + 1] = global_path.poses[idx].pose.position.y;
        x_ref[i * NX + 2] = unwrap_yaw(tf2::getYaw(global_path.poses[idx].pose.orientation));
    }
    // Compute u_ref
    for (int i = 0; i < N_Horizon; i++) {
        const double dx = x_ref[(i + 1) * NX + 0] - x_ref[i * NX + 0];
        const double dy = x_ref[(i + 1) * NX + 1] - x_ref[i * NX + 1];
        u_ref[i * NU + 0] = std::sqrt(dx * dx + dy * dy) / dt;  // linear_vel
        u_ref[i * NU + 1] = (x_ref[(i + 1) * NX + 2] - x_ref[i * NX + 2]) / dt; // angular_vel
    }
}

void MPCController::set_stage_cost_reference() {
    std::vector<double> yref(NY);
    std::vector<double> yref_e(NYN);

    for (int i = 0; i < N_Horizon; i++) {
        yref[0] = x_ref[i*NX+0];
		yref[1] = x_ref[i*NX+1];
		yref[2] = x_ref[i*NX+2];
		yref[3] = u_ref[i*NU+0];
		yref[4] = u_ref[i*NU+1];
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref.data());
    }

    yref_e[0] = x_ref[N_Horizon*NX+0];
	yref_e[1] = x_ref[N_Horizon*NX+1];
	yref_e[2] = x_ref[N_Horizon*NX+2];
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N_Horizon, "yref", yref_e.data());
}

void MPCController::control_loop() {
    if (!odom_received || global_path.poses.empty()) {
        return;
    }
    
    // Update state constraint
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0.data());
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0.data());

    update_global_path_index();
    compute_reference();
    
    set_stage_cost_reference();
    set_weights();

    // Solve OCP
    int status = haloball_acados_solve(acados_ocp_capsule);
    if (status != 0) {
        std::cout << "Solver failed with status: " << status << std::endl;
    }    
    publish_predicted_path(); 
    publish_control();
}

void MPCController::print_solution_info() {
    double elapsed_time;
    double total_cost;

    ocp_nlp_get(nlp_solver, "time_tot", &elapsed_time);
    ocp_nlp_get(nlp_solver, "cost_value", &total_cost);
    
    std::cout << "Total solution time: " << elapsed_time << " seconds." << std::endl;
    std::cout << "Total solution cost: " << total_cost << std::endl << std::endl;
}

void MPCController::publish_control() {
    // Extract control
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u.data());
    // Compute distance to goal
    const double goal_x = global_path.poses.back().pose.position.x;
    const double goal_y = global_path.poses.back().pose.position.y;
    const double dist_to_goal = std::hypot(x0[0] - goal_x, x0[1] - goal_y);

    // Check if goal reached and publish control
    geometry_msgs::msg::Twist cmd_vel;
    if (dist_to_goal < goal_tolerance && global_path_index >= global_path.poses.size()) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        // Print once after goal reached
        if (!goal_reached) {
            std::cout << "Goal Reached!" << std::endl;
            goal_reached = true;
        }
    } else {
        goal_reached = false;
        cmd_vel.linear.x = u[0];
        cmd_vel.angular.z = u[1];
        std::cout << "Control: v=" << u[0] << " m/s, ω=" << u[1] << " rad/s" << std::endl;
        print_solution_info();
    }
    cmd_vel_pub_->publish(cmd_vel);
}

void MPCController::publish_predicted_path() {
    nav_msgs::msg::Path predicted_path;
    predicted_path.header.frame_id = frame_id;
    predicted_path.header.stamp = now();

    for (int i = 0; i <= N_Horizon; i++) {
        // Extract predicted states
        double x_pred[NX];
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", x_pred);
        // Publish
        geometry_msgs::msg::PoseStamped pose;
        pose.header = predicted_path.header;
        pose.pose.position.x = x_pred[0];
        pose.pose.position.y = x_pred[1];
        pose.pose.orientation = createQuaternionMsgFromYaw(x_pred[2]);
        predicted_path.poses.push_back(pose);
    }
    predicted_path_pub_->publish(predicted_path);
}

double MPCController::unwrap_yaw(double current_yaw) {
    if (first_odom) {
        first_odom = false;
        prev_yaw = current_yaw;
        unwrapped_yaw = current_yaw;
        return current_yaw;
    }
    
    // Calculate the difference between current and previous yaw
    double delta = current_yaw - prev_yaw;
    
    // Handle the wrap-around cases
    if (delta > M_PI) {
        delta -= 2 * M_PI;
    } else if (delta < -M_PI) {
        delta += 2 * M_PI;
    }
    
    // Update unwrapped yaw
    unwrapped_yaw += delta;
    prev_yaw = current_yaw;
    
    return unwrapped_yaw;
}

geometry_msgs::msg::Quaternion MPCController::createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    return tf2::toMsg(quat);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCController>();
    rclcpp::Rate rate(MPCController::control_frequency);
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(node->get_node_base_interface());
        node->control_loop();
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}
