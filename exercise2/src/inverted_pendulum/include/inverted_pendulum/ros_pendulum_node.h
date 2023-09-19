#ifndef INVERTED_PENDULUM_ROS_PENDULUM_NODE_H_
#define INVERTED_PENDULUM_ROS_PENDULUM_NODE_H_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>

#include "inverted_pendulum/shared_context.h"
#include "inverted_pendulum_interfaces/srv/set_desired_position.hpp"
#include "inverted_pendulum_interfaces/srv/set_pid_constants.hpp"

class RosPendulumNode : public rclcpp::Node {
  // Shared context used to pass data to and from the real time thread
  std::shared_ptr<SharedContext> shared_context_;

  int64_t last_msg_sent_time_;

  int64_t decimated_msg_period_ns_;

  // Joint state publisher for all joint states (not decimated)
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr full_joint_state_publisher_;

  // Decimated joint state publisher for RViz
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr decimated_joint_state_publisher_;

  // Timer to periodically read from the shared context queue
  rclcpp::TimerBase::SharedPtr timer_;

  // Service to reset the pendulum back to its initial position
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;

  // Service to set the desired position (radians) of the pendulum
  rclcpp::Service<inverted_pendulum_interfaces::srv::SetDesiredPosition>::SharedPtr set_desired_position_service_;

  // Service to change the PID constants of the controller
  rclcpp::Service<inverted_pendulum_interfaces::srv::SetPIDConstants>::SharedPtr set_PID_constants_service_;

 public:
  /**
   * Creates a new ROS thread.
   */
  RosPendulumNode(const std::string& name, const std::shared_ptr<SharedContext> shared_context);

 private:
  void TimerCallback();

  void ResetPendulum(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /* request */,
    std::shared_ptr<std_srvs::srv::Empty::Response> /* response */
  ) {
    shared_context_->reset = true;
  }

  void SetDesiredPosition(
    const std::shared_ptr<inverted_pendulum_interfaces::srv::SetDesiredPosition::Request> request,
    std::shared_ptr<inverted_pendulum_interfaces::srv::SetDesiredPosition::Response> /* response */
  ) {
    shared_context_->desired_position.Set(request->desired_position);
  }

  void SetPIDConstants(
    const std::shared_ptr<inverted_pendulum_interfaces::srv::SetPIDConstants::Request> request,
    std::shared_ptr<inverted_pendulum_interfaces::srv::SetPIDConstants::Response> /* response */
  ) {
    shared_context_->pid_constants.Set(PIDConstants{request->kp, request->ki, request->kd});
  }
};

#endif
