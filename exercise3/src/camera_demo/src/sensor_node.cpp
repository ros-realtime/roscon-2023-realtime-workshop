// ROS2 subscriber node with two callbacks to the /camera topic

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;


class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode()
  : Node("subscriber_node")
  {
    // Create a subscription on the "camera" topic to log the image
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera", 10, std::bind(&SubscriberNode::data_logger, this, std::placeholders::_1));

    // Create a subscription on the "camera" topic to detect an object
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera", 10, std::bind(&SubscriberNode::object_detector, this, std::placeholders::_1));

    // Create a publisher on the "stop" topic
    publisher_ = this->create_publisher<std_msgs::msg::Empty>("stop", 10);
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

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
};

int main() {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<SubscriberNode>();

  // Put each subscriber in two different callback groups
  rclcpp::CallbackGroup::SharedPtr data_logger_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr object_detector_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create an executor to put the callback groups in
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_callback_group(data_logger_group, node->get_node_base_interface());
  executor.add_callback_group(object_detector_group, node->get_node_base_interface());

  // Run the node
  executor.spin();
  rclcpp::shutdown();
  return 0;
}