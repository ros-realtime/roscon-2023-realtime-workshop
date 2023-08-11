#ifndef INVERTED_PENDULUM_ROS_PENDULUM_NODE_H_
#define INVERTED_PENDULUM_ROS_PENDULUM_NODE_H_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>

#include "inverted_pendulum/shared_data.h"

class RosPendulumNode : public rclcpp::Node {
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr                               timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr           reset_service_;
  std::shared_ptr<SharedData>                                queue_;

 public:
  /**
   * Creates a new ROS thread.
   */
  RosPendulumNode(const std::string& name, const std::shared_ptr<SharedData> queue);

 private:
  void TimerCallback();
  void ResetPendulum(const std::shared_ptr<std_srvs::srv::Empty::Request> /* request */, std::shared_ptr<std_srvs::srv::Empty::Response> /* response */) { queue_->reset = true; }
};

#endif
