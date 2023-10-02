// ROS2 subscriber node with two callbacks to the /camera topic

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <list>
#include <thread>

#include "rclcpp/experimental/fifo_sched.hpp"

#include "camera_demo/sensor_node.hpp"

using namespace std::chrono_literals;

using ThreadTracer = cactus_rt::tracing::ThreadTracer;

SensorNode::SensorNode() : Node("subscriber_node")
{
  objdet_tracer_ = std::make_shared<ThreadTracer>("main_thread");
  logging_tracer_ = std::make_shared<ThreadTracer>("logging_thread");

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

SensorNode::~SensorNode() {
}

void SensorNode::data_logger(const sensor_msgs::msg::Image::SharedPtr msg)
{
  auto span = logging_tracer_->WithSpan("data_logger");

  // Print the image encoding
  RCLCPP_INFO(this->get_logger(), "Image encoding: %s", msg->encoding.c_str());

  // Print the image dimensions
  RCLCPP_INFO(this->get_logger(), "Image dimensions: %dx%d", msg->width, msg->height);
}

void SensorNode::object_detector(const sensor_msgs::msg::Image::SharedPtr msg)
{
  auto span = objdet_tracer_->WithSpan("object_detector");
  
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