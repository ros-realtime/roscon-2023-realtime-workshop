
#include "inverted_pendulum/ros_pendulum_node.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <functional>
#include <limits>

using namespace std::chrono_literals;
using namespace std::placeholders;

RosPendulumNode::RosPendulumNode(const std::string& name, const std::shared_ptr<SharedContext> shared_context)
    : rclcpp::Node(name),
      shared_context_(shared_context),
      last_msg_sent_time_(0),
      decimated_msg_period_ns_(10'000'000) {
  full_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_full", 10);
  decimated_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  timer_ = this->create_wall_timer(10ms, std::bind(&RosPendulumNode::TimerCallback, this));
  reset_service_ = this->create_service<std_srvs::srv::Empty>(
    "reset_pendulum", std::bind(&RosPendulumNode::ResetPendulum, this, _1, _2)
  );
  set_desired_position_service_ = this->create_service<inverted_pendulum_interfaces::srv::SetDesiredPosition>(
    "set_desired_position", std::bind(&RosPendulumNode::SetDesiredPosition, this, _1, _2)
  );
  set_PID_constants_service_ = this->create_service<inverted_pendulum_interfaces::srv::SetPIDConstants>(
    "set_PID_constants", std::bind(&RosPendulumNode::SetPIDConstants, this, _1, _2)
  );
}

void RosPendulumNode::TimerCallback() {
  OutputData data;

  // Empty the queue of all data
  while (true) {
    // Check if there is data in the queue
    if (shared_context_->PopData(data)) {
      // Construct a joint_state message for the pendulum position
      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp.sec = static_cast<int32_t>(data.timestamp.tv_sec);
      joint_state_msg.header.stamp.nanosec = static_cast<uint32_t>(data.timestamp.tv_nsec);
      joint_state_msg.name.push_back("joint_1");
      joint_state_msg.position.push_back(data.output_value);
      full_joint_state_publisher_->publish(joint_state_msg);

      // Check if the publishing period is met for the decimated publisher
      int64_t msg_timestamp = data.timestamp.tv_sec * 1'000'000'000 + data.timestamp.tv_nsec;
      if (msg_timestamp - last_msg_sent_time_ > decimated_msg_period_ns_) {
        decimated_joint_state_publisher_->publish(joint_state_msg);
        last_msg_sent_time_ = msg_timestamp;
      }
    } else {
      break;
    }
  }
}
