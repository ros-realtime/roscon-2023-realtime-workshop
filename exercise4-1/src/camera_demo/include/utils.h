#ifndef ROSCON_2023_REALTIME_WORKSHOP_UTILS_H_
#define ROSCON_2023_REALTIME_WORKSHOP_UTILS_H_
#include <cactus_rt/utils.h>

#include <chrono>

// A funtion to imply take up some time, pretending some processing has occured.
inline void WasteTime(std::chrono::microseconds duration) {
  const auto start = cactus_rt::NowNs();
  auto       duration_ns = duration.count() * 1000;
  while (cactus_rt::NowNs() - start < duration_ns) {
  }
}
#endif
