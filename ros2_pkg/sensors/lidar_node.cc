#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>


class LidarNode : public rclcpp::Node
{
public:
  LidarNode() : Node("lidar_node")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "lidar", 10, std::bind(&LidarNode::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    geometry_msgs::msg::Twist data;

    bool allMore = true;
    for (const auto& range : msg->ranges)
    {
      if (range < 1.0)
      {
        allMore = false;
        break;
      }
    }

    if (allMore)
    {
      data.linear.x = 0.5;
      data.angular.z = 0.0;
    }
    else
    {
      data.linear.x = 0.0;
      data.angular.z = 0.5;
    }
    std::ofstream file;  // Define 'file' here
    std::string package_path = ament_index_cpp::get_package_share_directory("ros2_pkg");
    std::string file_path = "/media/psf/Developer/Robotics/char-01/ros2_pkg/data";
    file.open(file_path, std::ios_base::app);
    for (const auto& range : msg->ranges)
    {
      file << range << " ";
    }
    file << "\n";
    file.close();

    pub_->publish(data);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}