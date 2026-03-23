#include <memory>
#include <vector>
#include <limits>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl_conversions/pcl_conversions.h>  
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>   

class PointCloudFilter : public rclcpp::Node
{
public:
  PointCloudFilter() : Node("pointcloud_filter")
  {
    // --- Define ROS Parameters ---
    // Topics
    std::string pointcloud_topic = this->declare_parameter("pointcloud_topic", "/your_pointcloud");
    std::string filtered_pointcloud_topic = this->declare_parameter("filtered_pointcloud_topic", "/your_pointcloud_filtered");
    std::string laser_scan_topic = this->declare_parameter("laser_scan_topic", "/scan");

    // Filtering Parameters
    pub_filtered_pointcloud_ = this->declare_parameter("publish_filtered_pointcloud", true);
    min_height_ = this->declare_parameter("min_height", -0.335 - 0.2339 + 0.05);
    max_height_ = this->declare_parameter("max_height", 0.5);
    filter_rings_ = this->declare_parameter("filter_rings", true);
    filter_height_ = this->declare_parameter("filter_height", true);
    
    // LaserScan Parameters
    pub_laserscan_ = this->declare_parameter("publish_laserscan", false);
    scan_angle_min_ = this->declare_parameter("scan_angle_min", -M_PI);
    scan_angle_max_ = this->declare_parameter("scan_angle_max", M_PI);
    scan_angle_increment_ = this->declare_parameter("scan_angle_increment", M_PI / 180.0 / 10.0);
    scan_range_min_ = this->declare_parameter("scan_range_min", 0.2);
    scan_range_max_ = this->declare_parameter("scan_range_max", 200.0);

    // --- Subscribers ---
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 10,
      std::bind(&PointCloudFilter::pointCloudCallback, this, std::placeholders::_1));

    // --- Publishers ---
    if (pub_filtered_pointcloud_)
    {
      filtered_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_pointcloud_topic, 10);
    }
    if (pub_laserscan_)
    {
      laser_scan_pub_  = this->create_publisher<sensor_msgs::msg::LaserScan>(laser_scan_topic, 10);
    }

    // Calculate the number of bins in the LaserScan message
    num_bins_ = static_cast<int>((scan_angle_max_ - scan_angle_min_) / scan_angle_increment_) + 1; // Add once, since that is ROS standard.
  
    // Debug Information
    std::cout << "pointcloud_filter node started" << std::endl;
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // --- Step 0: Detect if the incoming pointcloud has intensity data ---
    bool has_intensity = false;
    for (const auto & field : msg->fields)
    {
      if (field.name == "intensity")
      {
        has_intensity = true;
        break;
      }
    }

    // --- Step 1: Convert the incoming message to a unified point cloud type ---
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (has_intensity)
    {
      // Convert directly if intensity exists
      pcl::fromROSMsg(*msg, *input_cloud);
    }
    else
    {
      // Convert to pcl::PointXYZ first
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*msg, *temp_cloud);
      // Then copy to a pcl::PointXYZI cloud, setting intensity to 0
      for (const auto & pt : temp_cloud->points)
      {
        pcl::PointXYZI pt_i;
        pt_i.x = pt.x;
        pt_i.y = pt.y;
        pt_i.z = pt.z;
        pt_i.intensity = 0.0f;
        input_cloud->points.push_back(pt_i);
      }
      input_cloud->header = temp_cloud->header;
    }

    // --- Step 2: Remove LiDAR Rings (if enabled) ---
    pcl::PointCloud<pcl::PointXYZI>::Ptr intermediate_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (filter_rings_)
    {
      const int block_size = 16;
      size_t total_points = input_cloud->points.size();
      size_t num_blocks = total_points / block_size;
      intermediate_cloud->header = input_cloud->header;

      for (size_t i = 0; i < num_blocks; ++i)
      {
        if ((i % 2) == 0)  // Keep even-indexed blocks
        {
          for (int j = 0; j < block_size; ++j)
          {
            size_t idx = i * block_size + j;
            if (idx < input_cloud->points.size())
            {
              const auto & pt = input_cloud->points[idx];
              if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
              {
                intermediate_cloud->points.push_back(pt);
              }
            }
          }
        }
      }
    }
    else
    {
      intermediate_cloud = input_cloud;
    }

    // --- Step 3: Apply Height Filtering (if enabled) ---
    pcl::PointCloud<pcl::PointXYZI>::Ptr height_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (filter_height_)
    {
      for (const auto & pt : intermediate_cloud->points)
      {
        if (pt.z >= min_height_ && pt.z <= max_height_)
        {
          height_filtered_cloud->points.push_back(pt);
        }
      }
    }
    else
    {
      height_filtered_cloud = intermediate_cloud;
    }
    height_filtered_cloud->header = intermediate_cloud->header;

    // --- Step 4: Publish the Filtered 3D Point Cloud ---
    if(pub_filtered_pointcloud_)
    {
      // If the original cloud had intensity, publish as-is.
      // Otherwise, convert to a point cloud without intensity.
      if (has_intensity)
      {
        publishFilteredPointCloud(msg->header, height_filtered_cloud);
      }
      else
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_intensity(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_no_intensity->header = height_filtered_cloud->header;
        for (const auto & pt : height_filtered_cloud->points)
        {
          pcl::PointXYZ pt_xyz;
          pt_xyz.x = pt.x;
          pt_xyz.y = pt.y;
          pt_xyz.z = pt.z;
          cloud_no_intensity->points.push_back(pt_xyz);
        }
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_no_intensity, cloud_msg);
        cloud_msg.header = msg->header;
        filtered_pc_pub_->publish(cloud_msg);
      }
    }

    // --- Step 5: Create and Publish a 2D LaserScan Message ---
    // LaserScan generation uses only spatial coordinates, so intensity is irrelevant.
    if (pub_laserscan_) publishLaserScan(msg->header, height_filtered_cloud);
  }


  void publishFilteredPointCloud(const std_msgs::msg::Header &header, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header = header;
    filtered_pc_pub_->publish(cloud_msg);
  }

  void publishLaserScan(const std_msgs::msg::Header &header, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    if (cloud->points.empty())
      return;

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header = header;
    scan_msg.angle_min = scan_angle_min_;
    scan_msg.angle_max = scan_angle_max_;
    scan_msg.angle_increment = scan_angle_increment_;
    scan_msg.range_min = scan_range_min_;
    scan_msg.range_max = scan_range_max_;
    scan_msg.ranges.resize(num_bins_, std::numeric_limits<float>::infinity());

    for (const auto & pt : cloud->points)
    {
      float angle = std::atan2(pt.y, pt.x);
      float range = std::hypot(pt.x, pt.y);

      if (range < scan_range_min_ || range > scan_range_max_)
        continue;

      int index = static_cast<int>((angle - scan_msg.angle_min) / scan_msg.angle_increment);
      if (index >= 0 && index < num_bins_)
      {
        scan_msg.ranges[index] = std::min(scan_msg.ranges[index], range);
      }
    }

    laser_scan_pub_->publish(scan_msg);
  }

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;

  // Filtering parameters
  float min_height_;
  float max_height_;
  bool filter_rings_;
  bool filter_height_;
  bool pub_filtered_pointcloud_;
  bool pub_laserscan_;

  // LaserScan parameters
  float scan_angle_min_;
  float scan_angle_max_;
  float scan_angle_increment_;
  float scan_range_min_;
  float scan_range_max_;
  int num_bins_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}