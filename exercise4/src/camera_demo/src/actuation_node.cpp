// Listen to the /stop topic and stop the robot when a message is received

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <list>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "camera_demo/actuation_node.hpp"

using namespace std::chrono_literals;


ActuationNode::ActuationNode() : Node("actuation_node")
{
  actuation_tracer_ = std::make_shared<ThreadTracer>("actuation_thread");

  // Create a subscription on the "stop" topic
  subscription_ = this->create_subscription<std_msgs::msg::Empty>(
    "stop", 10, std::bind(&ActuationNode::stop_robot, this, std::placeholders::_1));
}

ActuationNode::~ActuationNode() {
}

void ActuationNode::stop_robot(const std_msgs::msg::Empty::SharedPtr)
{
  auto span = actuation_tracer_->WithSpan("stop_robot");
  // Stop the robot
  RCLCPP_INFO(this->get_logger(), "STOPPING ROBOT");
  bool stop = true;
}