#ifndef ROSCON_2023_REALTIME_WORKSHOP_SYSTEM_NODES_H_
#define ROSCON_2023_REALTIME_WORKSHOP_SYSTEM_NODES_H_

#include <cactus_rt/tracing.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#include "camera_demo_interfaces/msg/fake_image.hpp"

// Publishes at configurable frequency, always with real-time priority.
class ImagePublisherNode : public rclcpp::Node {
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<camera_demo_interfaces::msg::FakeImage>::SharedPtr publisher_;

 public:
  explicit ImagePublisherNode(double frequency_hz = 60.0);

 private:
  void TimerCallback();
};

void StartImagePublisherNode();
void JoinImagePublisherNode();

#endif
