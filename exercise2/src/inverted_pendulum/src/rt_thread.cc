#include "inverted_pendulum/rt_thread.h"

double RtThread::ReadSensor(int64_t cycle_time) {
  auto         span = Tracer().WithSpan("ReadSensor", "app");
  const double dt = static_cast<double>(cycle_time) / 1E9;

  const double g = 9.81;  // m / s^2

  // Calculate anglular acceleration
  double alpha = g / length_ * sin(current_position_);

  // Multiply by dt to get the change in angular velocity, and add the velocity command
  current_velocity_ += alpha * dt + velocity_command_;

  // Multiply by dt to get change in current position
  current_position_ += current_velocity_ * dt;

  // The pendulum has hit the floor :(
  if (abs(current_position_) > M_PI_2) {
    current_position_ = std::clamp(current_position_, -M_PI_2, M_PI_2);
    current_velocity_ = -0.4 * current_velocity_;  // Bounce!
  }

  return current_position_;
}

double RtThread::GetCommand(const double current_position, const double desired_position, int64_t cycle_time) {
  auto         span = Tracer().WithSpan("GetCommand", "app");
  const double dt = static_cast<double>(cycle_time) / 1E9;

  // Calculate error between desired and current position
  double error = desired_position - current_position;

  // Calculate integral of error with saturation
  error_sum_ = std::clamp(error_sum_ + error * dt, -100.0, 100.0);

  // Get the latest PID constants
  {
    auto pid_span = Tracer().WithSpan("GetPIDConstants", "app");
    pid_constants_ = shared_context_->pid_constants.Get();
  }

  // Calculate PID control law
  double position_command = pid_constants_.kp * error +
                            pid_constants_.ki * error_sum_ +
                            pid_constants_.kd * (error - prev_error_) / dt;

  // Update previous error
  prev_error_ = error;

  // Divide by dt to get the output as a velocity
  // Apply velocity limits with clamp
  double velocity_command = std::clamp(position_command / dt, -M_PI_4, M_PI_4);

  return velocity_command;
}

void RtThread::WriteCommand(const double output) {
  auto span = Tracer().WithSpan("WriteCommand", "app");
  velocity_command_ = output;
}

void RtThread::BeforeRun() {
  current_position_ = initial_position_;
  shared_context_->desired_position.Set(desired_position_);
  shared_context_->pid_constants.Set(pid_constants_);
}

bool RtThread::Loop(int64_t ellapsed_ns) noexcept {
  const int64_t cycle_time_ns = ellapsed_ns - prev_ns_;
  prev_ns_ = ellapsed_ns;

  if (shared_context_->reset) {
    current_position_ = initial_position_;
    current_velocity_ = 0;
    shared_context_->reset = false;
  }

  {
    auto span = Tracer().WithSpan("GetDesiredPosition", "app");
    desired_position_ = shared_context_->desired_position.Get();
  }

  const double current_position = ReadSensor(cycle_time_ns);
  const double output = GetCommand(current_position, desired_position_, cycle_time_ns);
  WriteCommand(output);

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);

  {
    auto span = Tracer().WithSpan("UpdateQueue", "app");
    shared_context_->data_queue.EmplaceData(ts, current_position);
  }

  LOG_INFO_LIMIT(std::chrono::milliseconds{100}, Logger(), "Controller output {}", output);

  // Loop forever
  return false;
}
