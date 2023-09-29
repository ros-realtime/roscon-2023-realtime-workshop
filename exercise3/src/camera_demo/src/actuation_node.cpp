// Listen to the /stop topic and stop the robot when a message is received

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <list>
#include <thread>
#include <cactus_rt/tracing.h>

#include "rclcpp/experimental/fifo_sched.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

using FileSink = cactus_rt::tracing::FileSink;
using TraceAggregator = cactus_rt::tracing::TraceAggregator;
using ThreadTracer = cactus_rt::tracing::ThreadTracer;


class ActuationNode : public rclcpp::Node
{
public:
  ActuationNode()
  : Node("actuation_node")
  {
    StartTracing("actuation_node", "actuation_node.json");
    trace_aggregator = nullptr;
    main_thread_tracer = std::make_shared<ThreadTracer>("main_thread");

    // Create a subscription on the "stop" topic
    subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "stop", 10, std::bind(&ActuationNode::stop_robot, this, std::placeholders::_1));

    // TODO: Omit this in the exercise
    sched_param sp;
    sp.sched_priority = MEDIUM;
    subscription_->sched_param(sp);
  }

  ~ActuationNode() {
    StopTracing();
  }

private:
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

  void stop_robot(const std_msgs::msg::Empty::SharedPtr)
  {
    auto span = main_thread_tracer->WithSpan("stop_robot");
    // Stop the robot
    RCLCPP_INFO(this->get_logger(), "STOPPING ROBOT");
    bool stop = true;
  }

  std::unique_ptr<TraceAggregator> trace_aggregator;
  std::shared_ptr<ThreadTracer> main_thread_tracer;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
  bool stop = false;
};

int main() {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<ActuationNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}