// Cactus RT
#include <cactus_rt/rt.h>

#include "inverted_pendulum/exercise2-2/ros_pendulum_node.h"
#include "inverted_pendulum/exercise2-2/rt_thread.h"
#include "inverted_pendulum/exercise2-2/shared_context.h"

using cactus_rt::App;

int main(int argc, char* argv[]) {
  // rclcpp can be init with rclcpp::SignalHandlerOptions::None, but this is not supported in Humble
  // https://github.com/ros2/rclcpp/issues/2020
  // Instead, we install a signal handler and use the init options to not shutdown on signal
  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options);

  // Set up cactus_rt signal handler
  // This is done after the rclcpp signal handler, and overrides it
  cactus_rt::SetUpTerminationSignalHandler();

  auto shared_context = std::make_shared<SharedContext>();
  auto node = std::make_shared<RosPendulumNode>("pendulum_node", shared_context);

  auto ros_thread = std::thread(
    [&]() {
      rclcpp::spin(node);
    }
  );

  // Repeatedly get PID constants to force lock contention
  auto get_pid_thread = std::thread(
    [&]() {
      while (true) {
        shared_context->pid_constants.Get();
      }
    }
  );

  cactus_rt::CyclicThreadConfig rt_thread_config;
  rt_thread_config.period_ns = 1'000'000;             // 1 ms loop
  rt_thread_config.SetFifoScheduler(80);              // Use FIFO scheduler with  thread priority 80
  rt_thread_config.tracer_config.trace_sleep = true;  // Trace the sleep duration
  auto rt_thread = std::make_shared<RtThread>(shared_context, rt_thread_config);

  App app("exercise2-2");
  app.RegisterThread(rt_thread);

  app.StartTraceSession("exercise2-2.perfetto");
  app.Start();

  // Stop trace after 2 minutes to prevent creating large files
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

  auto trace_timeout_thread = std::thread(
    [&]() {
      if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - begin).count() > 120)
        app.StopTraceSession();
    }
  );

  // We handle the termination signals and clean up before rclcpp::shutdown
  cactus_rt::WaitForAndHandleTerminationSignal();

  rt_thread->RequestStop();
  rt_thread->Join();

  // As part of shutdown, rclcpp will uninstall its own signal handler
  rclcpp::shutdown();
  ros_thread.join();

  return 0;
}
