#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <cmath>

class OdomSubscriber : public rclcpp::Node {
public:
  OdomSubscriber() : Node("odom_subscriber")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&OdomSubscriber::odom_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  double prev_yaw = 0.0, unwrapped_yaw = 0.0;
  bool is_first_message_ = true;

  // Helper function to calculate unwrapped yaw
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

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
      double yaw = tf2::getYaw(msg->pose.pose.orientation);  // [-pi, pi]
      double yaw_unwrapped = unwrap_yaw(yaw);

      std::cout << "yaw = " << yaw
                << ", yaw_unwrapped = " << yaw_unwrapped
                << std::endl;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomSubscriber>());
  rclcpp::shutdown();
  return 0;
}