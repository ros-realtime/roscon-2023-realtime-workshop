// ROS2 subscriber node with two callbacks to the /camera topic

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp/experimental/fifo_sched.hpp"

using namespace std::chrono_literals;


class SensorNode : public rclcpp::Node
{
public:
  SensorNode()
  : Node("subscriber_node")
  {
    // Create a subscription on the "camera" topic to log the image
    subscription_logger_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera", 10, std::bind(&SensorNode::data_logger, this, std::placeholders::_1));

    // Create a subscription on the "camera" topic to detect an object
    subscription_object_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera", 10, std::bind(&SensorNode::object_detector, this, std::placeholders::_1));

    // Create a publisher on the "stop" topic
    publisher_ = this->create_publisher<std_msgs::msg::Empty>("stop", 10);

    // TODO: Omit this in the exercise
    sched_param sp;
    sp.sched_priority = HIGH;
    subscription_object_->sched_param(sp);
  }

private:
  void data_logger(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Print the image encoding
    RCLCPP_INFO(this->get_logger(), "Image encoding: %s", msg->encoding.c_str());

    // Print the image dimensions
    RCLCPP_INFO(this->get_logger(), "Image dimensions: %dx%d", msg->width, msg->height);
  }

  void object_detector(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Detect an object in the image
    if (msg->data[0] == 1) {
      // Publish a message to /stop
      RCLCPP_INFO(this->get_logger(), "Object detected, stopping robot");

      // Create a new message
      auto msg = std::make_unique<std_msgs::msg::Empty>();

      // Publish the message
      publisher_->publish(std::move(msg));
    }
  }

  rclcpp::SensorNode<sensor_msgs::msg::Image>::SharedPtr subscription_logger_;
  rclcpp::SensorNode<sensor_msgs::msg::Image>::SharedPtr subscription_object_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
};

int main() {
  // OMIT IN EXERCISE
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<SensorNode>();

  // Create an executor to put the node into
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Run the executor
  executor.spin();
  rclcpp::shutdown();
  return 0;
}