#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

class LaserScanToPointCloudNode : public rclcpp::Node {
  public:
    LaserScanToPointCloudNode() : Node("laser_scan_to_point_cloud") {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&LaserScanToPointCloudNode::scan_callback, this, _1));
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_raw", 10);
    }

  private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2 cloud;
        projector_.projectLaser(*msg, cloud);
        publisher_->publish(cloud);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    laser_geometry::LaserProjection projector_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanToPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}