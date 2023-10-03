
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
    // should always use the try_* method in the hot path, as these do not allocate
    return queue_.try_emplace(timestamp, output_value);
  }

  bool PopData(OutputData& data) {
    WasteTime(std::chrono::microseconds(200));
    return queue_.try_dequeue(data);
  }
  void WasteTime(std::chrono::microseconds duration) {
    const auto start = cactus_rt::NowNs();
    auto       duration_ns = duration.count() * 1000;
    while (cactus_rt::NowNs() - start < duration_ns) {
    }
  }

 private:
  ReaderWriterQueue<OutputData> queue_ = ReaderWriterQueue<OutputData>(8'192);
};

#endif
