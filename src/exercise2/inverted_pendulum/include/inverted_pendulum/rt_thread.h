#ifndef INVERTED_PENDULUM_RT_THREAD_H_
#define INVERTED_PENDULUM_RT_THREAD_H_

#include <cactus_rt/rt.h>

#include <memory>

#include "inverted_pendulum/shared_context.h"

using cactus_rt::CyclicThread;

class RtThread : public CyclicThread {
  // Shared context used to pass data to and from the ROS thread
  std::shared_ptr<SharedContext> shared_context_;
  int64_t                        prev_ns_ = 0;

  // Pendulum properties
  const double initial_position_ = 0.6;  // Initial position of the pendulum in rad, 0 indicates top
  double       desired_position_ = 0.0;  // Desired position of the pendulum in rad, 0 indicates top
  const double length_ = 0.5;            // Length of the pendulum in meters
  double       current_position_;
  double       current_velocity_ = 0;  // Assume the pendulum starts from stationary
  double       velocity_command_ = 0;

  // Controller properties
  PIDConstants pid_constants_{5E-4, 0, 1E-5};
  double       error_sum_ = 0;
  double       prev_error_ = 0;

  // "Read" the sensor for pendulum angle by simulating it
  double ReadSensor(int64_t ellapsed_ns);

  // Get the velocity command for the motor with a PID controller
  double GetCommand(const double current_position, const double desired_position, int64_t ellapsed_ns);

  // Write the output of the PID to the robot
  void WriteCommand(const double output);

 public:
  RtThread(std::shared_ptr<SharedContext> shared_context, cactus_rt::CyclicThreadConfig config)
      : CyclicThread("RtThread", config),
        shared_context_(shared_context) {
  }

 protected:
  void BeforeRun() override;
  bool Loop(int64_t ellapsed_ns) noexcept final;
};

#endif
