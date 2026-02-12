#include <rclcpp/rclcpp.hpp>
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

// Function to generate a rounded rectangle trajectory
nav_msgs::msg::Path generate_rounded_rectangle_trajectory(double width, double height, double corner_radius, int points_per_corner, int points_per_side, std::string frame_id, rclcpp::Time stamp) {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = stamp;

    // Ensure the corner radius is not too large
    corner_radius = std::min(corner_radius, std::min(width / 2, height / 2));

    // Define the four corners of the rectangle (these are the centers of the rounded corners)
    geometry_msgs::msg::PoseStamped bottom_left, bottom_right, top_right, top_left;
    bottom_left.pose.position.x = -width / 2 + corner_radius; 
    bottom_left.pose.position.y = -height / 2 + corner_radius;

    bottom_right.pose.position.x = width / 2 - corner_radius;
    bottom_right.pose.position.y = -height / 2 + corner_radius;

    top_right.pose.position.x = width / 2 - corner_radius;
    top_right.pose.position.y = height / 2 - corner_radius;

    top_left.pose.position.x = -width / 2 + corner_radius;
    top_left.pose.position.y = height / 2 - corner_radius;

    // Helper function to generate a quarter-circle for rounded corners
    auto generate_corner = [&](double center_x, double center_y, double start_angle, double end_angle) {
        for (int i = 0; i <= points_per_corner; i++) {
            double angle = start_angle + i * (end_angle - start_angle) / points_per_corner;
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = stamp;
            pose.header.frame_id = frame_id;

            pose.pose.position.x = center_x + corner_radius * cos(angle);
            pose.pose.position.y = center_y + corner_radius * sin(angle);
            pose.pose.position.z = 0;
            pose.pose.orientation = createQuaternionMsgFromYaw(angle);

            path.poses.push_back(pose);
        }
    };

    // Generate the path for the rounded rectangle
    // 1. Bottom-right corner (from -π/2 to 0)
    generate_corner(bottom_right.pose.position.x, bottom_right.pose.position.y, -M_PI / 2, 0);

    // 2. Right side (vertical line between bottom-right and top-right)
    for (int i = 0; i <= points_per_side; i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;

        pose.pose.position.x = width / 2;
        pose.pose.position.y = -height / 2 + corner_radius + i * (height - 2 * corner_radius) / points_per_side;
        pose.pose.position.z = 0;
        pose.pose.orientation = createQuaternionMsgFromYaw(M_PI / 2);  // Facing upwards

        path.poses.push_back(pose);
    }

    // 3. Top-right corner (from 0 to π/2)
    generate_corner(top_right.pose.position.x, top_right.pose.position.y, 0, M_PI / 2);

    // 4. Top side (horizontal line between top-right and top-left)
    for (int i = 0; i <= points_per_side; i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;

        pose.pose.position.x = width / 2 - corner_radius - i * (width - 2 * corner_radius) / points_per_side;
        pose.pose.position.y = height / 2;
        pose.pose.position.z = 0;
        pose.pose.orientation = createQuaternionMsgFromYaw(M_PI);  // Facing left

        path.poses.push_back(pose);
    }

    // 5. Top-left corner (from π/2 to π)
    generate_corner(top_left.pose.position.x, top_left.pose.position.y, M_PI / 2, M_PI);

    // 6. Left side (vertical line between top-left and bottom-left)
    for (int i = 0; i <= points_per_side; i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;

        pose.pose.position.x = -width / 2;
        pose.pose.position.y = height / 2 - corner_radius - i * (height - 2 * corner_radius) / points_per_side;
        pose.pose.position.z = 0;
        pose.pose.orientation = createQuaternionMsgFromYaw(-M_PI / 2);  // Facing downwards

        path.poses.push_back(pose);
    }

    // 7. Bottom-left corner (from π to 3π/2)
    generate_corner(bottom_left.pose.position.x, bottom_left.pose.position.y, M_PI, 3 * M_PI / 2);

    // 8. Bottom side (horizontal line between bottom-left and bottom-right)
    for (int i = 0; i <= points_per_side; i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;

        pose.pose.position.x = -width / 2 + corner_radius + i * (width - 2 * corner_radius) / points_per_side;
        pose.pose.position.y = -height / 2;
        pose.pose.position.z = 0;
        pose.pose.orientation = createQuaternionMsgFromYaw(0);  // Facing right

        path.poses.push_back(pose);
    }

    tf2::Quaternion quat;
    tf2::fromMsg(path.poses[0].pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
                
    return path;
}

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher() : Node("trajectory_publisher")
    {   
        // Declare parameters
        this->declare_parameter<std::string>("frame_id", "odom"); // default is odom
        frame_id = this->get_parameter("frame_id").as_string();

        trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/trajectory", 
            rclcpp::QoS(10).transient_local());
        
        // Create a timer to periodically publish the trajectory
        timer_ = this->create_wall_timer(
            1000ms,  // Publish every second
            std::bind(&TrajectoryPublisher::publish_trajectory, this));
    }

private:
    void publish_trajectory()
    {
        // circle parameters
        int circle_points = 300;
        double circle_radius = 0.5;
        double circle_rotation = -1.57;    // rotation angle in radians

        // fig8 parameters
        int fig8_points = 700;
        double fig8_radius = 1.0;

        // oval parameters
        int oval_points = 300;
        double oval_semi_major = 0.5;  // width
        double oval_semi_minor = 1.0;  // height
        double oval_rotation = -1.57;    // rotation angle in radians
        
        // rounded rectangle        
        int points_per_corner = 100;
        int points_per_side = 100;
        double corner_radius = 3.0;
        double width = 10.0;
        double height = 10.0;
        
        // Get current time for the path timestamp
        auto now = this->now();
        
        // Generate trajectory (uncomment the one you want to use)
        // nav_msgs::msg::Path path = generate_circle_trajectory(circle_radius, circle_rotation, circle_points, frame_id, now);
        nav_msgs::msg::Path path = generate_figure_eight_trajectory(fig8_radius, fig8_points, frame_id, now);
        // nav_msgs::msg::Path path = generate_oval_trajectory(oval_semi_major, oval_semi_minor, oval_rotation, oval_points, frame_id, now);
        // nav_msgs::msg::Path path = generate_rounded_rectangle_trajectory(width, height, corner_radius, points_per_corner, points_per_side, frame_id, now);
        
        // Publish the trajectory
        trajectory_pub_->publish(path);
        
        // For debug
        // RCLCPP_INFO(this->get_logger(), "Published trajectory with %zu points", path.poses.size());
    }

    // Declare node variables
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string frame_id;
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