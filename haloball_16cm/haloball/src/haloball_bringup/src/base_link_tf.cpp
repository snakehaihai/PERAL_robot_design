#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class BaseLinkTF : public rclcpp::Node
{
public:
    BaseLinkTF() : Node("base_link_tf")
    {
        // Declare parameter with default value of 0.0m
        this->declare_parameter<double>("lidar_to_base_link_distance", 0.0);
        D_ = this->get_parameter("lidar_to_base_link_distance").as_double();

        // Subscriber to lidar odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&BaseLinkTF::odomCallback, this, std::placeholders::_1));

        // Publisher for base_link odometry
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // tf broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract body pose
        const double x = msg->pose.pose.position.x;
        const double y = msg->pose.pose.position.y;
        const double z = msg->pose.pose.position.z;

        const auto & q = msg->pose.pose.orientation;

        // Convert quaternion -> roll, pitch, yaw
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // base_link position (same formulas as Python)
        const double base_link_x = x - D_ * std::sin(pitch);
        const double base_link_y = y + D_ * std::sin(roll);
        const double base_link_z = z - D_ * std::cos(roll) * std::cos(pitch);

        // Publish base_link odometry message
        nav_msgs::msg::Odometry base_link_odom;
        base_link_odom.header.stamp = this->get_clock()->now();
        base_link_odom.header.frame_id = "camera_init";
        base_link_odom.child_frame_id = "base_link";

        base_link_odom.pose.pose.position.x = base_link_x;
        base_link_odom.pose.pose.position.y = base_link_y;
        base_link_odom.pose.pose.position.z = base_link_z;
        base_link_odom.pose.pose.orientation = q;

        // Copy original twist (velocities)
        base_link_odom.twist = msg->twist;

        odom_pub_->publish(base_link_odom);

        // Publish base_link TF: camera_init -> base_link
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = base_link_odom.header.stamp;
        transform.header.frame_id = "camera_init";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = base_link_x;
        transform.transform.translation.y = base_link_y;
        transform.transform.translation.z = base_link_z;
        transform.transform.rotation = q;

        tf_broadcaster_->sendTransform(transform);
    }

    // Declare node variables
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double D_;  // Distance between LiDAR and the base of shell.
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseLinkTF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
