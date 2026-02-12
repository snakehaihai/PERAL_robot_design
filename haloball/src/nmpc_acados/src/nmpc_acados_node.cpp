#include "nmpc_acados_node.h"

#define NX          HALOBALL_NX  // Number of state variables
#define NU          HALOBALL_NU  // Number of control inputs

#define NP          HALOBALL_NP  // Number of parameters in cost functions

#define NY          HALOBALL_NY  // Number of measurements/references on nodes 0..N - 1
#define NYN         HALOBALL_NYN // Number of measurements/references on node N

#define N           HALOBALL_N   // Number of intervals in the horizon

#define TF 5                  // (s) predict horizon   
#define Ts (TF / (double)N)   // (s) sampling time

// Declare robot variable
const double wheel_radius = 0.017; // meters
const double wheel_separation = 0.125; // meters
// Declare ROS parameters
string frame_id;
double weight_x, weight_y, weight_yaw, weight_right_wheel_angular_vel, weight_left_wheel_angular_vel;
// Declare MPC variable
double x0[NX]; // current state x0 = [x, y, yaw]
bool odom_received = false;

// Function declaration.
void odomCallback(const nav_msgs::msg::Odometry& msg);
void pathCallback(const nav_msgs::msg::Path& msg);
bool is_target(double cur_x, double cur_y, double goal_x, double goal_y);

// Function definition

// Function to create quaternion from yaw using tf2 (can take in unwrapped yaw)
geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    return tf2::toMsg(quat);
}

// Function to unwrap yaw
double prev_yaw = 0.0, unwrapped_yaw = 0.0;
bool is_first_message_ = true;

double unwrap_yaw(double current_yaw) {
    if (is_first_message_) {
      is_first_message_ = false;
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

// Get current state x0 = [x, y, yaw]
void odomCallback(const nav_msgs::msg::Odometry& msg) {
	x0[0] = msg.pose.pose.position.x;
	x0[1] = msg.pose.pose.position.y;
	double odom_yaw = tf2::getYaw(msg.pose.pose.orientation);
	x0[2] = unwrap_yaw(odom_yaw);

	odom_received = true;
}

nav_msgs::msg::Path path;
void pathCallback(const nav_msgs::msg::Path& msg) 
{ 
	path = msg;
	// Debug
	// std::cout << "Received new trajectory with " << msg.poses.size() << " poses." << std::endl;
}

bool is_target(double cur_x, double cur_y, double goal_x, double goal_y)
{
    if(abs(cur_x - goal_x) < 0.05 && abs(cur_y - goal_y) < 0.05)
    {
        return true;
    }
    else return false;
}

void publishPredictedPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher,
                         const std::vector<std::vector<double>>& state_output,
                         rclcpp::Node::SharedPtr node) {
    nav_msgs::msg::Path predict_path;
    predict_path.header.frame_id = frame_id;
    predict_path.header.stamp = node->now();
    
    for (int i = 0; i <= N; i++) {
        geometry_msgs::msg::PoseStamped pred_pose;
        pred_pose.header = predict_path.header;
        pred_pose.pose.position.x = state_output[i][0];
        pred_pose.pose.position.y = state_output[i][1];
        pred_pose.pose.orientation = createQuaternionMsgFromYaw(state_output[i][2]);
        predict_path.poses.push_back(pred_pose);
    }
    
    publisher->publish(predict_path);
}

void printSolutionInfo(ocp_nlp_solver* nlp_solver) {
    double elapsed_time;
    double total_cost;
    
    ocp_nlp_get(nlp_solver, "time_tot", &elapsed_time);
    ocp_nlp_get(nlp_solver, "cost_value", &total_cost);
    
    std::cout << "Total solution time: " << elapsed_time << " seconds." << std::endl;
    std::cout << "Total solution cost: " << total_cost << std::endl << std::endl;
}

// Main Function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("nmpc_acados_node");

    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, odomCallback);
    auto path_sub = node->create_subscription<nav_msgs::msg::Path>(
        "/trajectory", 1, pathCallback);

    auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto predict_pub = node->create_publisher<nav_msgs::msg::Path>("/predict_path", 10);

  	// ROS PARAM
	node->declare_parameter<string>("frame_id", "odom");
  	node->declare_parameter<double>("weight_x", 0.0);
  	node->declare_parameter<double>("weight_y", 0.0);
  	node->declare_parameter<double>("weight_yaw", 0.0);
  	node->declare_parameter<double>("weight_right_wheel_angular_vel", 0.0);
  	node->declare_parameter<double>("weight_left_wheel_angular_vel", 0.0);
  	
	node->get_parameter("frame_id", frame_id);
  	node->get_parameter("weight_x", weight_x);
  	node->get_parameter("weight_y", weight_y);
  	node->get_parameter("weight_yaw", weight_yaw);
  	node->get_parameter("weight_right_wheel_angular_vel", weight_right_wheel_angular_vel);
  	node->get_parameter("weight_left_wheel_angular_vel", weight_left_wheel_angular_vel);
	
	std::cout << "weight_x: " << weight_x << " weight_y: " << weight_y << " weight_yaw: " << weight_yaw << " weight_right_wheel_angular_vel: " 
				<< weight_right_wheel_angular_vel << " weight_left_wheel_angular_vel: " << weight_left_wheel_angular_vel << std::endl;
    
    rclcpp::Rate r(10);
	int count = 0;

/***************************************** Initialise solver *****************************************/
    haloball_solver_capsule *acados_ocp_capsule = haloball_acados_create_capsule();
    int status = haloball_acados_create(acados_ocp_capsule);
    if (status) {
        std::cerr << "haloball_acados_create() returned status " << status << ". Exiting." << std::endl;
        return 1;
    }
    ocp_nlp_config *nlp_config = haloball_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = haloball_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = haloball_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = haloball_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = haloball_acados_get_nlp_solver(acados_ocp_capsule);
/******************************************************************************************************/

/********************************************* Main Loop **********************************************/
    while(rclcpp::ok())
    {	
		if (!odom_received)
		{
			std::cout << "Waiting for odometry..." << std::endl;
			rclcpp::spin_some(node);
			r.sleep();
			continue;
		}
		if (path.poses.size() == 0)
		{
			std::cout << "Waiting for trajectory..." << std::endl;
			rclcpp::spin_some(node);
			r.sleep();
			continue;
		}

		double goal_x = path.poses[path.poses.size()-1].pose.position.x;
		double goal_y = path.poses[path.poses.size()-1].pose.position.y;

		/* Set current state x0 = [x, y, yaw] as initial condition constraint
		This fixes the initial state at stage 0 to the current odometry values
		by setting both lower and upper bounds to x0 (equality constraint) */
		ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
		ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);
		
		// ref variables
		vector<double> ref_states((N + 1) * NX);
		vector<double> right_wheel_angular_velocities(N);
		vector<double> left_wheel_angular_velocities(N);

        // x_ref
		for (int i = 0; i <= N; i++)
		{
			if (count + i >= path.poses.size())
			{
				ref_states[i * NX + 0] = path.poses[path.poses.size()-1].pose.position.x;
				ref_states[i * NX + 1] = path.poses[path.poses.size()-1].pose.position.y;
				double ref_yaw = tf2::getYaw(path.poses[path.poses.size()-1].pose.orientation);
				ref_states[i * NX + 2] = unwrap_yaw(ref_yaw);


			}
			else
			{
				ref_states[i * NX + 0] = path.poses[count+i].pose.position.x;
				ref_states[i * NX + 1] = path.poses[count+i].pose.position.y;
				double ref_yaw  = tf2::getYaw(path.poses[count+i].pose.orientation);
				ref_states[i * NX + 2] = unwrap_yaw(ref_yaw);
			}
		}

		// u_ref
		for (int i = 0; i < N; i++) {
			// dx and dy
			double dx = ref_states[(i + 1) * NX + 0] - ref_states[i * NX + 0];
			double dy = ref_states[(i + 1) * NX + 1] - ref_states[i * NX + 1];
			// ref_v (linear_velocity)
			double v = sqrt(dx * dx + dy * dy) / Ts;

			// ref_w (angular_velocity)
			double yaw_dot = ref_states[(i + 1) * NX + 2] - ref_states[i * NX + 2];

			// ref_w (angular_velocity)
			double w = yaw_dot / Ts;

			// convert v, w to left/right_wheel_angular_velocities
			right_wheel_angular_velocities[i] = (v + 0.5 * (w * wheel_separation)) / wheel_radius;
			left_wheel_angular_velocities[i] = (v - 0.5 * (w * wheel_separation)) / wheel_radius;
		}

		// yref and yrefN
		vector<double> yref(NY);
		vector<double> yref_N(NYN);

		// set yref
		for (int i = 0; i < N; i++)
		{
			yref[0] = ref_states[i * NX + 0];       // ref_x
			yref[1] = ref_states[i * NX + 1];       // ref_y 
			yref[2] = ref_states[i * NX + 2];       // ref_yaw
			yref[3] = right_wheel_angular_velocities[i];                // ref_v
			yref[4] = left_wheel_angular_velocities[i];        // ref_w
			ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref.data());
		}

		// set yref_N (y_ref at terminal)
		yref_N[0] = ref_states[N * NX + 0];       // ref_x at terminal
		yref_N[1] = ref_states[N * NX + 1];       // ref_y at terminal
		yref_N[2] = ref_states[N * NX + 2];       // ref_yaw at terminal
		ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_N.data());

		// W and WN
		double Weights[NY * NY] = {0};
		double Weights_N[NYN * NYN] = {0};

		// set W and WN
		Weights[0 * NY + 0] = weight_x;  // w_x 
		Weights[1 * NY + 1] = weight_y;  // w_y 
		Weights[2 * NY + 2] = weight_yaw; // w_yaw
		Weights[3 * NY + 3] = weight_right_wheel_angular_vel; // w_v
		Weights[4 * NY + 4] = weight_left_wheel_angular_vel; // w_w

		Weights_N[0 * NYN + 0] = weight_x;  // w_x_N
		Weights_N[1 * NYN + 1] = weight_y;  // w_y_N
		Weights_N[2 * NYN + 2] = weight_yaw; // w_yaw_N
		

		for (int i = 0; i < N; i++) {
			ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", Weights);
		}
		ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", Weights_N);
		
		// solve ocp
		int solver_status = haloball_acados_solve(acados_ocp_capsule);
		if (solver_status != 0) {
		    std::cerr << "haloball_acados_solve() failed with status " << solver_status << "." << std::endl;
		} else {
		    std::cout << "haloball_acados_solve() succeeded." << std::endl;
		}
		
		// Print solution time and cost
        printSolutionInfo(nlp_solver);
	
		// get solutions
		vector<vector<double>> control_output(N, vector<double>(NU));
		vector<vector<double>> state_output(N + 1, vector<double>(NX));

		for (int i = 0; i < N; i++)
		{
		    double u_out[NU];
		    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", (void *)u_out);
		    control_output[i] = vector<double>(u_out, u_out + NU);

		    double x_out[NX];
		    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", (void *)x_out);
		    state_output[i] = vector<double>(x_out, x_out + NX);
		}
		double x_out_n[NX];
		ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, N, "x", (void *)x_out_n);
		state_output[N] = vector<double>(x_out_n, x_out_n + NX);
    
/******************************************************************************************************/

		publishPredictedPath(predict_pub, state_output, node);

       	// publish u_out
		geometry_msgs::msg::Twist vel;
		bool goal = is_target(x0[0], x0[1], goal_x, goal_y) && count >= path.poses.size();
		if(goal)
		{
			vel.linear.x = 0;
			vel.angular.z = 0;
		}
		else
		{	
			double right_wheel_angular_vel = control_output[0][0];
   			double left_wheel_angular_vel = control_output[0][1];

			vel.linear.x = (right_wheel_angular_vel + left_wheel_angular_vel) * 0.5 * wheel_radius;
			vel.angular.z = (right_wheel_angular_vel - left_wheel_angular_vel) * (wheel_radius / wheel_separation);
		}
		vel_pub->publish(vel);

		std::cout << "Control output: linear = " << vel.linear.x
        			<< ", angular = " << vel.angular.z << std::endl;

        rclcpp::spin_some(node);
        r.sleep();
		count++;
    }
/********************************************* Loop  end **********************************************/
    
/***************************************** FREE_ACADOS_SOLVER *****************************************/
    int status_free = haloball_acados_free(acados_ocp_capsule);
    if (status_free) {
        std::cerr << "haloball_acados_free() returned status " << status_free << "." << std::endl;
    }
    status_free = haloball_acados_free_capsule(acados_ocp_capsule);
    if (status_free) {
        std::cerr << "haloball_acados_free_capsule() returned status " << status_free << "." << std::endl;
    }
/******************************************************************************************************/

    rclcpp::shutdown();
    return 0;
}