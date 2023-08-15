#ifndef INVERTED_PENDULUM_SHARED_CONTEXT_H_
#define INVERTED_PENDULUM_SHARED_CONTEXT_H_

#include <cactus_rt/rt.h>
#include <readerwriterqueue.h>

#include <atomic>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using moodycamel::ReaderWriterQueue;

struct OutputData {
  struct timespec timestamp;
  double          output_value = 0.0;
  OutputData() = default;
  OutputData(struct timespec t, double o) : timestamp(t), output_value(o) {}
};

// Shared context between the RT thread and the ROS thread
class SharedContext {
  using CountData = struct {
    uint32_t successful_messages;
    uint32_t total_messages;
  };

  using AtomicMessageCount = std::atomic<CountData>;
  AtomicMessageCount message_count_;
  static_assert(AtomicMessageCount::is_always_lock_free);

 public:
  SharedContext(){};

  std::atomic<bool>             reset = false;
  ReaderWriterQueue<OutputData> queue = ReaderWriterQueue<OutputData>(8'192);

  /**
   * This method should only be called by one consumer (thread). It pushes data
   * to be logged into a file for later consumption.
   */
  bool EmplaceData(struct timespec timestamp, double output_value) noexcept {
    // should always use the try_* method in the hot path, as these do not allocate
    const bool success = queue.try_emplace(timestamp, output_value);
    if (success) {
      IncrementMessageCount(1);
    } else {
      IncrementMessageCount(0);
    }
    return success;
  }

 private:
  void IncrementMessageCount(uint32_t successful_message_count) noexcept {
    auto                old_count = message_count_.load();
    decltype(old_count) new_count;

    do {
      new_count = old_count;
      new_count.successful_messages += successful_message_count;
      new_count.total_messages += 1;
    } while (!message_count_.compare_exchange_weak(old_count, new_count));
  }
};

#endif
