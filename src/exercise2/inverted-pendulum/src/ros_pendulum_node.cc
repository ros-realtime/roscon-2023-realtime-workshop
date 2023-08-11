
#include "inverted_pendulum/ros_pendulum_node.h"

#include <chrono>
#include <functional>
using namespace std::chrono_literals;
using namespace std::placeholders;

RosPendulumNode::RosPendulumNode(const std::string& name, const std::shared_ptr<SharedData> queue)
    : rclcpp::Node(name), queue_(queue) {
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  timer_ = this->create_wall_timer(10ms, std::bind(&RosPendulumNode::TimerCallback, this));
  reset_service_ = this->create_service<std_srvs::srv::Empty>("reset_pendulum", std::bind(&RosPendulumNode::ResetPendulum, this, _1, _2));
}

// TODO subsample messages to not overload RViz
// Also consider outputting all messages to a new topic for bag purposes
void RosPendulumNode::TimerCallback() {
  OutputData data;

  while (true) {
    if (queue_->try_dequeue(data)) {
      sensor_msgs::msg::JointState joint_state_msg;
      // joint_state_msg.header.stamp = this->get_clock()->now();
      joint_state_msg.header.stamp.sec = data.timestamp.tv_sec;
      joint_state_msg.header.stamp.nanosec = data.timestamp.tv_nsec;
      joint_state_msg.name.push_back("joint_1");
      joint_state_msg.position.push_back(data.output_value);
      joint_state_publisher_->publish(joint_state_msg);
    } else {
      break;
    }
  }
}
