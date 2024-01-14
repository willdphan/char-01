// Include necessary headers for timing and functional utilities.
#include <chrono>
#include <functional> 

// : =  inheritance
// :: = scope or "inside of." after third :: you can declare variable name
// & = used to get the "address" or location of a specific function 

// Include necessary headers for the ROS2 C++ client library and the standard string message type.
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Use the namespace for chrono literals, allowing us to use time literals like "1s" for 1 second.
using namespace std::chrono_literals;

// Define a class "HelloWorldPubNode" that inherits from the rclcpp::Node class.
class HelloWorldPubNode : public rclcpp::Node
{
  public: 
    // Constructor for the HelloWorldPubNode class.
    HelloWorldPubNode() : Node("hello_world_pub_node") 
    {
      // Create a publisher that publishes to the "hello_world" topic, with a queue size of 10.
			// using inherited node, it calls func with params
			// std_msgs::msg::String is a notation used in ROS (Robot Operating System). 
			// std_msgs is a package containing standard message types. 
			// Within that, msg::String represents a message type for transmitting text (strings).
    	publisher_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);

      // Create a timer that calls the "publish_hello_world" function every 1 second.
			// We're using a tool (create_wall_timer) inside this object. Every 1 second, it looks inside the 
			// HelloWorldPubNode class (we created) and activates the publish_hello_world function inside of that class.
			// bind tells which action (or function) to perform when the timer goes off (every 1s)
      timer_ = this->create_wall_timer(1s, std::bind(&HelloWorldPubNode::publish_hello_world, this));
    }

  private:
    // Define a function to publish a "Hello, World" message with a counter.
    void publish_hello_world()
    {
      // Create a new standard string message.
      auto message = std_msgs::msg::String();

      // Set the data field of the message to "Hello, World" followed by the current counter value.
      message.data = "Hello, World  " + std::to_string(counter_);

      // Publish the message using the publisher.
			// publisher_ is an instance (or an object) of some class defined in ROS library
			// -> = "Go inside" the mailbox (publisher_ variable) and drop the message inside
      publisher_->publish(message);

      // Increment the counter.
      counter_++;
    }

    // Declare a shared pointer to a publisher for standard string messages.
		// Looking inside rclcpp, there's a tool (or class) called Publisher. 
		// Inside this Publisher, we're specifically interested in a version that deals 
		// with text messages (std_msgs::msg::String).
		// After deciding what kind of messages our Publisher will handle, 
		// we're choosing to hold or manage this Publisher using a type called SharedPtr. 
		// A SharedPtr is a smart pointer that helps manage memory automatically.
		// _publisher is the name we've given to our specific instance or object of that Publisher.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // Declare a shared pointer to a timer.
    rclcpp::TimerBase::SharedPtr timer_;

    // Declare a counter variable initialized to 0
		// size_t is a type that comes from the C++ standard library
		// It's an unsigned integer type that's commonly used to represent sizes and counts in C++
    size_t counter_ = 0;
};

// Entry point of the program.
// main func returns int
// char = a single letter or symbol.
// char * = an arrow pointing to a character
// char * argv[] is a way to hold the program's name and any extra information (arguments) 
// you give to the program when you run it
int main(int argc, char * argv[])
{
  // Initialize the ROS2 client library.
  rclcpp::init(argc, argv);

  // Create an instance of the HelloWorldPubNode class and keep it spinning.
  // This means the node will keep running, allowing callbacks (like the timer) to be triggered.
	// We're telling ROS to keep our program running and listening for events, 
	// using a the HelloWorldPubNode class we just made.
  rclcpp::spin(std::make_shared<HelloWorldPubNode>());

  // Shutdown the ROS2 client library before exiting the program.
  rclcpp::shutdown();
  
  // Return 0 indicating successful execution.
  return 0;
}
