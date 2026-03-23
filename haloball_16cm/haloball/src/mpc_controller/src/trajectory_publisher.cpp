#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

// Function to create quaternion from yaw using tf2
geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion quat;
  quat.setRPY(0, 0, yaw);
  return tf2::toMsg(quat);
}

// Function to generate a circle trajectory
nav_msgs::msg::Path generate_circle_trajectory(double radius, double rotation_angle, int points, std::string frame_id, rclcpp::Time stamp) {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = stamp;

    // Center of the circle
    double x_shift = 0.0;
    double y_shift = 0.5;

    for (int i = 0; i < points; i++) {
        double angle = i * 2 * M_PI / points;

        // Circle parametric equations
        double x_circle = radius * cos(angle);
        double y_circle = radius * sin(angle);

        // Apply rotation according to rotation_angle
        double x_rot = x_circle * cos(rotation_angle) - y_circle * sin(rotation_angle);
        double y_rot = x_circle * sin(rotation_angle) + y_circle * cos(rotation_angle);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;

        pose.pose.position.x = x_rot + x_shift;
        pose.pose.position.y = y_rot + y_shift;
        pose.pose.position.z = 0;  // Assuming flat ground
        pose.pose.orientation = createQuaternionMsgFromYaw(angle);

        path.poses.push_back(pose);
    }

    return path;
}

// Function to generate a figure-eight trajectory
nav_msgs::msg::Path generate_figure_eight_trajectory(double radius, int points, std::string frame_id, rclcpp::Time stamp) {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = stamp;

    for (int i = 0; i < points; i++) {
        double angle = i * 2 * M_PI / points; // angle goes from 0 to 2*PI for figure eight
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;

        // Figure-eight parametric equations
        pose.pose.position.x = radius * sin(angle);
        pose.pose.position.y = radius * sin(angle) * cos(angle);
        pose.pose.position.z = 0;  // Assuming flat ground

        // Compute orientation towards the next point on the trajectory
        double next_angle = (i+1) * 2 * M_PI / points;
        double next_x = radius * sin(next_angle);
        double next_y = radius * sin(next_angle) * cos(next_angle);
        double yaw = atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x);
        pose.pose.orientation = createQuaternionMsgFromYaw(yaw);

        path.poses.push_back(pose);
    }

    return path;
}

// Function to generate an oval trajectory
nav_msgs::msg::Path generate_oval_trajectory(double semi_major_axis, double semi_minor_axis, double rotation_angle, int points, std::string frame_id, rclcpp::Time stamp) {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = stamp;

    // Center of the oval
    double x_shift = 0.0;
    double y_shift = 0.5;

    for (int i = 0; i < points; i++) {
        double angle = i * 2 * M_PI / points;  // angle goes from 0 to 2*PI
        
        // Oval parametric equations
        double x_oval = semi_major_axis * cos(angle);
        double y_oval = semi_minor_axis * sin(angle);
        
        // Apply rotation according to rotation_angle
        double x_rot = x_oval * cos(rotation_angle) - y_oval * sin(rotation_angle);
        double y_rot = x_oval * sin(rotation_angle) + y_oval * cos(rotation_angle);
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;
        
        // Apply center offset
        pose.pose.position.x = x_rot + x_shift;
        pose.pose.position.y = y_rot + y_shift;
        pose.pose.position.z = 0;  // Assuming flat ground
        
        // Compute derivative for orientation (tangent to the oval)
        double dx_dtheta = -semi_major_axis * sin(angle);
        double dy_dtheta = semi_minor_axis * cos(angle);
        
        // Rotate the derivative
        double dx_rot = dx_dtheta * cos(rotation_angle) - dy_dtheta * sin(rotation_angle);
        double dy_rot = dx_dtheta * sin(rotation_angle) + dy_dtheta * cos(rotation_angle);
        
        // Yaw is the angle of the tangent vector
        double yaw = atan2(dy_rot, dx_rot);
        pose.pose.orientation = createQuaternionMsgFromYaw(yaw);
        
        path.poses.push_back(pose);
    }

    return path;
}

nav_msgs::msg::Path generate_straight_path (
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const std::string& frame_id,
    const rclcpp::Time& stamp) 
{   
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = stamp;

    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    double yaw = atan2(dy, dx);
    int num_points = std::max(2, static_cast<int>(distance / 0.02));

    for (int i = 0; i < num_points; ++i) {
        double ratio = static_cast<double>(i) / (num_points - 1);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;

        pose.pose.position.x = start.pose.position.x + ratio * dx;
        pose.pose.position.y = start.pose.position.y + ratio * dy;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = createQuaternionMsgFromYaw(yaw);
        path.poses.push_back(pose);
    }

    return path;
}

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher() : Node("trajectory_publisher")
    {   
        // Declare parameters
        frame_id = this->declare_parameter("frame_id", "odom"); // default is odom
        trajectory_type = this->declare_parameter<std::string>("trajectory_type", "fig8"); // default is fig8
        // Publisher
        global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1);

        // Subscriber
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&TrajectoryPublisher::odom_callback, this, std::placeholders::_1)
        );

        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&TrajectoryPublisher::goal_pose_callback, this, std::placeholders::_1)
        );
        
        // Create a timer to periodically publish the trajectory
        timer_ = this->create_wall_timer(
            1000ms,  // Publish every second
            std::bind(&TrajectoryPublisher::publish_trajectory, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::ConstPtr& msg) {
        current_pose.header = msg->header;
        current_pose.pose = msg->pose.pose;
        has_odom = true;
    }

    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr& msg) {
        goal_pose = *msg;
        has_goal = true;
        if (has_odom) {
            path = generate_straight_path(current_pose, goal_pose, frame_id, this->now());
        }
    }

    void publish_trajectory()
    {
        // circle parameters
        int circle_points = 300;
        double circle_radius = 0.5;
        double circle_rotation = -1.57;    // rotation angle in radians

        // fig8 parameters
        int fig8_points = 300;
        double fig8_radius = 1.0;

        // oval parameters
        int oval_points = 300;
        double oval_semi_major = 0.5;  // width
        double oval_semi_minor = 1.0;  // height
        double oval_rotation = -1.57;    // rotation angle in radians
        
        // Get current time for the path timestamp
        auto now = this->now();
        
        // Generate trajectory
        if (trajectory_type == "circle") {
            path = generate_circle_trajectory(circle_radius, circle_rotation, circle_points, frame_id, now);
        }
        else if (trajectory_type == "fig8") {
            path = generate_figure_eight_trajectory(fig8_radius, fig8_points, frame_id, now);
        }
        else if (trajectory_type == "oval") {
            path = generate_oval_trajectory(oval_semi_major, oval_semi_minor, oval_rotation, oval_points, frame_id, now);
        }
        else {
            if (!has_odom || !has_goal) {
                return;
            }
        }
        // Publish the trajectory
        global_path_pub_->publish(path);
    }

    // Declare node variables
    nav_msgs::msg::Path path;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::PoseStamped goal_pose;
    bool has_odom = false, has_goal = false;

    std::string frame_id, trajectory_type;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Create and spin the node
    auto node = std::make_shared<TrajectoryPublisher>();

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}