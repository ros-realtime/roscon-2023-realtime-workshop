#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <list>
#include <thread>
#include <cactus_rt/tracing.h>

#include "rclcpp/rclcpp.hpp"
#include "camera_demo/publisher_node.hpp"
#include "camera_demo/sensor_node.hpp"
#include "camera_demo/actuation_node.hpp"

using FileSink = cactus_rt::tracing::FileSink;
using TraceAggregator = cactus_rt::tracing::TraceAggregator;
using ThreadTracer = cactus_rt::tracing::ThreadTracer;

std::unique_ptr<TraceAggregator> trace_aggregator;
std::shared_ptr<ThreadTracer> main_thread_tracer;

void StartTracing(const char* app_name, const char* filename) {
  // Enable the tracing.
  cactus_rt::tracing::EnableTracing();

  // Create the trace aggregator that will pop the queues and write the events to sinks.
  trace_aggregator = std::make_unique<TraceAggregator>(app_name);

  // Create the file sink so the data aggregated by the TraceAggregator will be written to somewhere.
  auto file_sink = std::make_shared<FileSink>(filename);
  trace_aggregator->RegisterSink(file_sink);

  trace_aggregator->Start();
}

void StopTracing() {
  cactus_rt::tracing::DisableTracing();

  trace_aggregator->RequestStop();
  trace_aggregator->Join();
  trace_aggregator = nullptr;  // Destroy the trace aggregator and free all resources.
}

int main() {
  StartTracing("camera_demo", "camera_demo.perfetto3");

  rclcpp::init(0, nullptr);
  auto publisher_node = std::make_shared<PublisherNode>();
  auto sensor_node = std::make_shared<SensorNode>();
  auto actuation_node = std::make_shared<ActuationNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(sensor_node);
  executor.add_node(actuation_node);
  executor.spin();
  rclcpp::shutdown();

  StopTracing();
  return 0;
}