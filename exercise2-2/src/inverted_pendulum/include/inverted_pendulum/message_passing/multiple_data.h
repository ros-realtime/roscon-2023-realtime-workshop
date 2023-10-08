#ifndef INVERTED_PENDULUM_MULTIPLE_DATA_H_
#define INVERTED_PENDULUM_MULTIPLE_DATA_H_

#include <cactus_rt/mutex.h>

#include <mutex>

struct PIDConstants {
  double kp;  // Proportional gain
  double ki;  // Integral gain
  double kd;  // Derivative gain
};

struct MultipleData {
  void Set(PIDConstants pid_constants) {
    const std::scoped_lock lock(pid_constant_mutex_);
    pid_constants_ = pid_constants;
  }

  PIDConstants Get() {
    const std::scoped_lock lock(pid_constant_mutex_);
    return pid_constants_;
  }

 private:
  // Exercise 2-2: Change this mutex to a priority inheritance mutex
  // A priority inheritance mutex implementation is available as cactus_rt::mutex
  using mutex = cactus_rt::mutex;
  mutex pid_constant_mutex_;

  PIDConstants pid_constants_;
};

#endif
