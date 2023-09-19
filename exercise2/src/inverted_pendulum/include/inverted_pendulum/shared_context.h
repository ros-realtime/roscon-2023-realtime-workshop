#ifndef INVERTED_PENDULUM_SHARED_CONTEXT_H_
#define INVERTED_PENDULUM_SHARED_CONTEXT_H_

#include <cactus_rt/rt.h>

#include <atomic>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "inverted_pendulum/message_passing/data_queue.h"
#include "inverted_pendulum/message_passing/multiple_data.h"
#include "inverted_pendulum/message_passing/single_data.h"

// Shared context between the RT thread and the ROS thread
class SharedContext {
 public:
  SharedContext(){};

  // Used to reset the pendulum to its initial position and velocity
  std::atomic<bool> reset = false;

  // Used to set the desired position (in radians) for the pendulum
  SingleData desired_position;

  // Used to get and set PID constants
  MultipleData pid_constants;

  // Used to pass pendulum positions
  DataQueue data_queue;

 private:
};

#endif
