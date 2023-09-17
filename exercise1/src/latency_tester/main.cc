#include <cactus_rt/rt.h>

cactus_rt::CyclicThreadConfig CreateRtThreadConfig() {
  cactus_rt::CyclicThreadConfig config;
  config.SetFifoScheduler(80);

  config.tracer_config.trace_loop = true;
  config.tracer_config.trace_wakeup_latency = true;

  return config;
}

class RtThread : public cactus_rt::CyclicThread {
 public:
  RtThread() : CyclicThread("RtThread", CreateRtThreadConfig()) {}

 protected:
  bool Loop(int64_t /* ellapsed_ns */) noexcept final {
    return false;
  }
};

int main() {
  auto rt_thread = std::make_shared<RtThread>();

  cactus_rt::App app;
  app.RegisterThread(rt_thread);

  cactus_rt::SetUpTerminationSignalHandler();

  app.StartTraceSession("data.perfetto");
  app.Start();

  cactus_rt::WaitForAndHandleTerminationSignal();

  app.RequestStop();
  app.Join();

  return 0;
}
