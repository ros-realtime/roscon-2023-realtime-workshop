#ifndef INVERTED_PENDULUM_SINGLE_DATA_H_
#define INVERTED_PENDULUM_SINGLE_DATA_H_

#include <atomic>
#include <mutex>

struct SingleData {
  // Exercise2-1: Remove the lock from this function
  void Set(const double value) {
    value_ = value;
  }

  // Exercise2-1: Remove the lock from this function
  double Get() {
    return value_;
  }

 private:
  // Exercise2-1: Remove the lock and replace the double with a std::atomic<double>
  static_assert(std::atomic<double>::is_always_lock_free);
  std::atomic<double>     value_;
};

#endif
