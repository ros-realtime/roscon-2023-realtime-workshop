#ifndef INVERTED_PENDULUM_ROS_PENDULUM_NODE_H_
#define INVERTED_PENDULUM_ROS_PENDULUM_NODE_H_

#include <cactus_rt/rt.h>
#include <readerwriterqueue.h>

// ROS
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using moodycamel::ReaderWriterQueue;

struct OutputData {
  double timestamp = 0;
  double output_value = 0.0;

  OutputData() = default;
  OutputData(double t, double o) : timestamp(t), output_value(o) {}
};

class RosPendulumNode : public rclcpp::Node {
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr                               timer_;
  ReaderWriterQueue<OutputData>                              queue_;

  using CountData = struct {
    uint32_t successful_messages;
    uint32_t total_messages;
  };

  using AtomicMessageCount = std::atomic<CountData>;
  AtomicMessageCount message_count_;
  static_assert(AtomicMessageCount::is_always_lock_free);

 public:
  /**
   * Creates a new ROS thread.
   */
  RosPendulumNode(const std::string& name);

  /**
   * This method should only be called by one consumer (thread). It pushes data
   * to be logged into a file for later consumption.
   */
  bool EmplaceData(double timestamp, double output_value) noexcept;

 private:
  void TimerCallback();
  void IncrementMessageCount(uint32_t successful_message_count) noexcept;
};

#endif
