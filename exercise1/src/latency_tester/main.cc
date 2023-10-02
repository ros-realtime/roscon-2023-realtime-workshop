#include <cactus_rt/rt.h>

#include <argparse/argparse.hpp>
#include <chrono>
#include <thread>

const char* kAppName = "LatencyTester";

cactus_rt::CyclicThreadConfig CreateRtThreadConfig(uint32_t index) {
  cactus_rt::CyclicThreadConfig config;

  config.SetOtherScheduler();

  // Uncomment he following line to use the real-time scheduler
  // config.SetFifoScheduler(80);

  config.cpu_affinity = {index};

  config.tracer_config.trace_loop = true;
  config.tracer_config.trace_overrun = true;
  config.tracer_config.trace_sleep = true;
  config.tracer_config.trace_wakeup_latency = true;

  return config;
}

void WasteTime(std::chrono::microseconds duration) {
  const auto start = cactus_rt::NowNs();
  auto       duration_ns = duration.count() * 1000;
  while (cactus_rt::NowNs() - start < duration_ns) {
  }
}

class RtThread : public cactus_rt::CyclicThread {
 public:
  explicit RtThread(uint32_t index) : CyclicThread(
                                        std::string("RtThread") + std::to_string(index),
                                        CreateRtThreadConfig(index)
                                      ) {}

 protected:
  bool Loop(int64_t /* ellapsed_ns */) noexcept final {
    WasteTime(std::chrono::microseconds(200));
    return false;
  }
};

argparse::ArgumentParser ParseArgs(int argc, char** argv) {
  argparse::ArgumentParser program(kAppName);

  program.add_argument("-j", "--threads")
    .scan<'d', uint32_t>()
    .default_value(static_cast<uint32_t>(2))
    .help("Number of concurrent RT threads");

  program.add_argument("-t", "--time")
    .scan<'i', int32_t>()
    .default_value(10)
    .help("Number of seconds to test latency for");

  program.add_argument("-f", "--file")
    .default_value(std::string("exercise1.perfetto"))
    .help("Perfetto trace file to write to");

  try {
    program.parse_args(argc, argv);
  } catch (const std::invalid_argument& err) {
    std::cerr << err.what() << "\n";
    std::cerr << program;
    std::exit(1);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << "\n";
    std::cerr << program;
    std::exit(1);
  }

  return program;
}

int main(int argc, char** argv) {
  auto program = ParseArgs(argc, argv);
  auto time_in_seconds = program.get<int32_t>("--time");
  auto threads = program.get<uint32_t>("--threads");
  auto filename = program.get("--file");

  if (threads > std::thread::hardware_concurrency()) {
    std::cerr << "error: specified threads is greater than num cpu " << std::thread::hardware_concurrency() << "\n";
    std::exit(1);
  }

  cactus_rt::App app(kAppName);

  for (uint32_t i = 0; i < threads; i++) {
    app.RegisterThread(std::make_shared<RtThread>(i));
  }

  cactus_rt::SetUpTerminationSignalHandler();

  app.StartTraceSession(filename.c_str());
  app.Start();

  auto time_limit = std::chrono::seconds(time_in_seconds);
  auto start = std::chrono::steady_clock::now();

  std::cerr << "Testing latency for " << time_in_seconds << " seconds with " << threads << " threads...\n";

  while (!cactus_rt::HasTerminationSignalBeenReceived() && (std::chrono::steady_clock::now() - start < time_limit)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  app.RequestStop();
  app.Join();

  std::cerr << "Latency testing complete. Trace file available at: " << filename << "\n";

  return 0;
}
