#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <iostream>
#include <math.h>

const float DEFAULT_WHEEL_RADIUS = 12.5 / 100; // centimeters to meters

class SpeedCalcNode : public rclcpp::Node 
{
  public: 
		SpeedCalcNode(): Node("speed_calc_node") 
		{
			this->declare_parameter<double>("wheel_radius", DEFAULT_WHEEL_RADIUS);
			rpm_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
				"rpm", 10, std::bind(&SpeedCalcNode::calculate_and_pub_speed, this, std::placeholders::_1)
			);
			speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
					"speed", 10
			);
			std::cout << "Speed Calc Node Is Running..." << std::endl;
		}

	private: 
		void calculate_and_pub_speed(const std_msgs::msg::Float64 & rpm_msg) const 
		{
			auto speed_msg = std_msgs::msg::Float64();

			float wheel_radius_param=DEFAULT_WHEEL_RADIUS;
			this->get_parameter("wheel_radius", wheel_radius_param);

			// Speed [m/s] = RPM [rev/min] * Wheel_Circumference [meters/rev] / 60 [seconds/min]
			speed_msg.data = rpm_msg.data * (2 * wheel_radius_param * M_PI) / 60;

			speed_publisher_->publish(speed_msg);
		}

		rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rpm_subscription_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedCalcNode>());
  rclcpp::shutdown();
  
  return 0;
}