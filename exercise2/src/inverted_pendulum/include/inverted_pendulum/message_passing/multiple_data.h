
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
  using mutex = cactus_rt::mutex;
  // using mutex = std::mutex;
  mutex pid_constant_mutex_;

  PIDConstants pid_constants_;
};
