#ifndef INVERTED_PENDULUM_SINGLE_DATA_H_
#define INVERTED_PENDULUM_SINGLE_DATA_H_

#include <atomic>
#include <mutex>

struct SingleData {
  static_assert(std::atomic<double>::is_always_lock_free);

  void Set(const double value) {
    value_ = value;
  }

  double Get() {
    return value_;
  }

 private:
  std::atomic<double> value_;
};

#endif
