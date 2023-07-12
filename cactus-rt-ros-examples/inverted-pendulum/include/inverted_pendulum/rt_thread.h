#ifndef CACTUS_RT_EXAMPLES_MESSAGE_PASSING_EXAMPLE_RT_THREAD_H_
#define CACTUS_RT_EXAMPLES_MESSAGE_PASSING_EXAMPLE_RT_THREAD_H_

#include <cactus_rt/rt.h>

#include <memory>

#include "inverted_pendulum/data_logger_thread.h"

using cactus_rt::CyclicThread;
using cactus_rt::schedulers::Fifo;

class RtThread : public CyclicThread<>
{
  std::shared_ptr<DataLogger> data_logger_;
  size_t max_iterations_;
  size_t iterations_ = 0;
  int64_t prev_ns_ = 0;

  // Pendulum properties
  const double initial_position_ = 0.3; // Initial position of the pendulum in rad, 0 indicates top
  const double length_ = 0.5;           // Length of the pendulum in meters
  double current_position_;
  double current_velocity_ = 0; // Assume the pendulum starts from stationary
  double velocity_command_ = 0;

  // Controller properties
  const double kp_ = 5E-4;
  const double ki_ = 0;
  const double kd_ = 1E-5;
  double error_sum_ = 0;
  double prev_error_ = 0;

  double ReadSensor(int64_t ellapsed_ns);
  double GetCommand(const double current_position, const double desired_position, int64_t ellapsed_ns);
  void WriteCommand(const double output);

public:
  RtThread(std::shared_ptr<DataLogger> data_logger, int period_ns, size_t max_iterations = 30000)
      : CyclicThread<>("RtThread", period_ns, Fifo::Config{80 /* priority */}),
        data_logger_(data_logger),
        max_iterations_(max_iterations)
  {
  }

protected:
  void BeforeRun() override;
  bool Loop(int64_t ellapsed_ns) noexcept final;
};

#endif
