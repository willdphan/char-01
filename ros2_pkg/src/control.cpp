// subscribes to the topic where the predictions are published, and uses these predictions to control your robot in the Gazebo simulation

// node will publish geometry_msgs/Twist messages to the /cmd_vel topic based on the predictions

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sstream>
#include <vector>

class ControlNode : public rclcpp::Node
{
public:
    ControlNode()
        : Node("control_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "prediction", 10, std::bind(&ControlNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        geometry_msgs::msg::Twist twist;

        std::vector<double> prediction;
        std::istringstream iss(msg->data);
        double number;
        while (iss >> number) {
            prediction.push_back(number);
        }

        if (!prediction.empty()) {
            twist.linear.x = prediction[0];
        }
        twist.angular.z = 0.0;

        publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}