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
  /**
   * This method should only be called by one consumer (thread). It pushes data
   * to be logged into a file for later consumption.
   */
  bool EmplaceData(struct timespec timestamp, double output_value) noexcept {
    std::scoped_lock lock(mutex_);
    queue_.emplace(timestamp, output_value);
    return true;
  }

  bool PopData(OutputData& data) {
    std::scoped_lock lock(mutex_);
    WasteTime(std::chrono::microseconds(200));
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
  std::queue<OutputData> queue_;
  std::mutex             mutex_;
};

#endif
