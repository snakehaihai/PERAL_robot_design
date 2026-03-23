#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Declare global variable to keep code neat
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr real_path_pub;
nav_msgs::msg::Path path;

// Callback function for odometry messages
void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg->header;        // Copy timestamp and frame_id
    pose_stamped.pose = odom_msg->pose.pose;       // Copy pose

    path.poses.push_back(pose_stamped);
    real_path_pub->publish(path);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("odom_path_publisher");

    // Parameters (customizable from launch)
    node->declare_parameter<std::string>("frame_id", "odom");
    node->declare_parameter<std::string>("odom_topic", "/odom");
    node->declare_parameter<std::string>("odom_path_topic", "/odom_path");

    const auto frame_id  = node->get_parameter("frame_id").as_string();
    const auto odom_topic = node->get_parameter("odom_topic").as_string();
    const auto odom_path_topic = node->get_parameter("odom_path_topic").as_string();

    // Publisher for the path
    real_path_pub = node->create_publisher<nav_msgs::msg::Path>(
        odom_path_topic, 10);

    // Subscribe to odometry
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, odomCallback);

    // Initialize path message
    path.header.frame_id = frame_id;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
