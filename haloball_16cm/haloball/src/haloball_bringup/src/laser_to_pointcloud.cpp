#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

class LaserToPointCloud : public rclcpp::Node
{
public:
    LaserToPointCloud() : Node("laser_to_pointcloud")
    {
        // Setup tf2 to transform 
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create publisher
        scan_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/scan_pointcloud", 10);
        
        // Create subscriber
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LaserToPointCloud::scanCallback, this, std::placeholders::_1));
        
        // Initialize projector
        projector_ = std::make_unique<laser_geometry::LaserProjection>();

        // Debug Information
        std::cout << "laser_to_pointcloud node started" << std::endl;
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_in)
    {
        sensor_msgs::msg::PointCloud2 scan_pointcloud;
        
        // Convert LaserScan to PointCloud2
        // projector_->projectLaser(*scan_in, scan_pointcloud);
        
        // Copy header and publish
        // scan_pointcloud.header = scan_in->header;

        // scan_pointcloud_pub_->publish(scan_pointcloud);

        // Wait for the transform to be available
        // Note: laser_geometry handles the lookup, but a specific check/timeout prevents errors if the transform is slightly delayed.
        // Transform laser_frame -> odom
        if (!tf_buffer_->canTransform("odom", scan_in->header.frame_id, 
                                        scan_in->header.stamp, rclcpp::Duration(1, 0))) {
            RCLCPP_WARN(this->get_logger(), "Transform from %s to odom not available yet", 
                        scan_in->header.frame_id.c_str());
            return;
        }

        projector_->transformLaserScanToPointCloud("odom", *scan_in, scan_pointcloud, *tf_buffer_);
        
        // Publish the result
        scan_pointcloud_pub_->publish(scan_pointcloud);
    }

    // Declare node variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pointcloud_pub_;
    std::unique_ptr<laser_geometry::LaserProjection> projector_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserToPointCloud>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}