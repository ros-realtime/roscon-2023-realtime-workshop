// ROS2 Node that publishes a camera image at 30Hz

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/experimental/fifo_sched.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "camera_demo/publisher_node.hpp"

using namespace std::chrono_literals;


PublisherNode::PublisherNode() : Node("publisher_node")
{
  publishing_tracer_ = std::make_shared<ThreadTracer>("publishing_thread");

  // Create a publisher on the "camera" topic
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);

  // Create a timer that calls the publish function every 33ms
  timer_ = this->create_wall_timer(33ms, std::bind(&PublisherNode::publish, this));

  // TODO: Omit this in the exercise
  sched_param sp;
  sp.sched_priority = HIGH;
  timer_->sched_param(sp);
}

PublisherNode::~PublisherNode() {
}

void PublisherNode::publish()
{
  auto span = publishing_tracer_->WithSpan("publish");

  // Create a new image message
  auto msg = std::make_unique<sensor_msgs::msg::Image>();

  // Fill the message header
  msg->header.stamp = this->now();
  msg->header.frame_id = "camera";

  // Set the image dimensions
  msg->height = 480;
  msg->width = 640;

  // Set the image encoding
  msg->encoding = "rgb8";

  // Set the image data
  msg->data.resize(msg->height * msg->width * 3);

  // Publish the message
  publisher_->publish(std::move(msg));
}