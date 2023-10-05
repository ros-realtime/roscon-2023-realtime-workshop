#ifndef INVERTED_PENDULUM_SINGLE_DATA_H_
#define INVERTED_PENDULUM_SINGLE_DATA_H_

#include <atomic>
#include <mutex>

// Exercise 2-2 uses multiple_data.h
// Do not modify this file for exercise 2-2

struct SingleData {
  void Set(const double value) {
    value_ = value;
  }

  double Get() {
    return value_;
  }

 private:
  static_assert(std::atomic<double>::is_always_lock_free);
  std::atomic<double> value_;
};

#endif
