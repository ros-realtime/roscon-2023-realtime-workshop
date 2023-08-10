
#include "inverted_pendulum/ros_pendulum_node.h"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

RosPendulumNode::RosPendulumNode(const std::string& name)
    : rclcpp::Node(name) {
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  timer_ = this->create_wall_timer(10ms, std::bind(&RosPendulumNode::TimerCallback, this));
}

void RosPendulumNode::TimerCallback() {
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->get_clock()->now();
  ;
  joint_state_msg.name.push_back("joint_1");
  joint_state_msg.position.push_back(0.0);
  joint_state_publisher_->publish(joint_state_msg);
}

bool RosPendulumNode::EmplaceData(double timestamp, double output_value) noexcept {
  // should always use the try_* method in the hot path, as these do not allocate
  const bool success = queue_.try_emplace(timestamp, output_value);
  if (success) {
    IncrementMessageCount(1);
  } else {
    IncrementMessageCount(0);
  }
  return success;
}

void RosPendulumNode::IncrementMessageCount(uint32_t successful_message_count) noexcept {
  auto                old_count = message_count_.load();
  decltype(old_count) new_count;

  do {
    new_count = old_count;
    new_count.successful_messages += successful_message_count;
    new_count.total_messages += 1;
  } while (!message_count_.compare_exchange_weak(old_count, new_count));
}
