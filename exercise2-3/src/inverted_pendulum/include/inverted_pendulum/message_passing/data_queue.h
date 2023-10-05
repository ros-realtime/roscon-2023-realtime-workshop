#ifndef INVERTED_PENDULUM_DATA_QUEUE_H_
#define INVERTED_PENDULUM_DATA_QUEUE_H_

#include <cactus_rt/utils.h>
#include <readerwriterqueue.h>

#include <mutex>
#include <queue>

using moodycamel::ReaderWriterQueue;

struct OutputData {
  struct timespec timestamp;
  double          output_value = 0.0;
  OutputData() = default;
  OutputData(struct timespec t, double o) : timestamp(t), output_value(o) {}
};

struct DataQueue {
  // Exercise 2-3: Remove the lock in this function and use the lockfree queue's
  // try_emplace(timestamp, output_value) method (guaranteed to never allocate)
  // try_emplace will return true if the element was emplaced, false otherwise.
  bool EmplaceData(struct timespec timestamp, double output_value) noexcept {
    std::scoped_lock lock(mutex_);
    queue_.emplace(timestamp, output_value);
    return true;
  }

  // Exercise 2-3: Remove the lock in this function and use the lockfree queue's
  // try_dequeue(data) method
  // try_dequeue will attempt to dequeue an element; if the queue is empty, it
  // returns false.
  bool PopData(OutputData& data) {
    std::scoped_lock lock(mutex_);
    WasteTime(std::chrono::microseconds(200));  // Simulate doing some work
    if (queue_.size() == 0) {
      return false;
    }
    data = queue_.front();
    queue_.pop();
    return true;
  }

  void WasteTime(std::chrono::microseconds duration) {
    const auto start = cactus_rt::NowNs();
    auto       duration_ns = duration.count() * 1000;
    while (cactus_rt::NowNs() - start < duration_ns) {
    }
  }

 private:
  // Exercise 2-3: Change this queue into a lockfree queue
  // A lockfree queue implementation is available as moodycamel::ReaderWriterQueue
  // Remember to pre-allocate memory for the queue.
  // A queue size of 8'192 can be used for this exercise
  std::queue<OutputData> queue_;
  std::mutex             mutex_;
};

#endif
