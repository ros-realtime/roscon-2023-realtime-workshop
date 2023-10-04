#ifndef INVERTED_PENDULUM_SINGLE_DATA_H_
#define INVERTED_PENDULUM_SINGLE_DATA_H_

#include <atomic>
#include <mutex>

struct SingleData {
  static_assert(std::atomic<double>::is_always_lock_free);

  // Exercise2-1: Remove the lock from this function
  void Set(const double value) {
    std::scoped_lock lock(mutex_);
    value_ = value;
  }

  // Exercise2-1: Remove the lock from this function
  double Get() {
    std::scoped_lock lock(mutex_);
    return value_;
  }

 private:
  // Exercise2-1: Remove the lock and replace the double with a std::atomic<double>
  std::mutex mutex_;
  double     value_;
};

#endif
