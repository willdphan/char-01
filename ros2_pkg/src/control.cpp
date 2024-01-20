// Converts the prediction to a Twist message: The callback function takes a std_msgs::msg::String message (the prediction), converts the data to a double, and uses this to create a geometry_msgs::msg::Twist message. The linear.x field of the Twist message is set to the prediction, and the angular.z field is set to 0.0.

// Publishes the Twist message to the /cmd_vel topic: The callback function publishes the Twist message to the /cmd_vel topic. This message can be used to control the movement of a robot. 

// ControlNode is responsible for converting predictions (in the form of std_msgs::msg::String messages on the /prediction topic) into movement commands (in the form of geometry_msgs::msg::Twist messages on the /cmd_vel topic). However, it doesn't actually move the robot itself - that would be the responsibility of another node that subscribes to the /cmd_vel topic.


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

    // Convert the prediction from string to double
    double prediction = std::stod(msg->data);

    // Use the prediction to set the linear.x field of the twist message
    twist.linear.x = prediction;
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