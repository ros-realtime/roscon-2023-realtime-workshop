
#include <atomic>

struct SingleData {
  static_assert(std::atomic<double>::is_always_lock_free);
  std::atomic<double> value;
};