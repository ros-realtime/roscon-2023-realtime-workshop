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

  // TODO: Supply these for callback groups
  rclcpp::SensorNode<sensor_msgs::msg::Image>::SharedPtr subscription_logger_;
  rclcpp::SensorNode<sensor_msgs::msg::Image>::SharedPtr subscription_object_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
};

int main() {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<SubscriberNode>();

  // Put each subscriber in two different callback groups
  rclcpp::CallbackGroup::SharedPtr data_logger_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr object_detector_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);  // OMIT IN EXERCISE

  // Create an executor to put the callback groups in
  rclcpp::executors::SingleThreadedExecutor data_logging_executor;
  rclcpp::executors::SingleThreadedExecutor object_detector_executor;  // OMIT IN EXERCISE
  data_logging_executor.add_callback_group(data_logger_group, node->get_node_base_interface());
  object_detector_executor.add_callback_group(object_detector_group, node->get_node_base_interface());  // OMIT IN EXERCISE

  // OMIT IN EXERCISE
  // Create thread for object detector executor
  std::thread object_detector_thread([&object_detector_executor]() {
    // Give this thread real-time priority
    sched_param sch;
    sch.sched_priority = 50;
    if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
      perror("sched_setscheduler failed");
      exit(-1);
    }
    object_detector_executor.spin();
  });

  // OMIT IN EXERCISE
  // Run the data logger in the main thread, without realtime priority
  data_logging_executor.spin();
  return 0;
}