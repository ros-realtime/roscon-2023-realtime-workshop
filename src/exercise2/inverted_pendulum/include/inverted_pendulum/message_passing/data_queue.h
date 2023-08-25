
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

    // std::scoped_lock lock(mutex_);
    // queue_.emplace(timestamp, output_value);
    // return true;
  }

  bool PopData(OutputData& data) {
    return queue_.try_dequeue(data);

    // std::scoped_lock lock(mutex_);
    // if (queue_.size() == 0) {
    //   return false;
    // }
    // data = queue_.front();
    // queue_.pop();
    // return true;
  }

 private:
  ReaderWriterQueue<OutputData> queue_ = ReaderWriterQueue<OutputData>(8'192);
  // std::queue<OutputData> queue_;
  // std::mutex             mutex_;
};
