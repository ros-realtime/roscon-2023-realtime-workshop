// Listen to the /stop topic and stop the robot when a message is received

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cactus_rt/tracing.h>

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

  // OMIT IN EXERCISE
  // Set the priority of this thread to be real-time
  sched_param sch;
  sch.sched_priority = 50;
  if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
    perror("sched_setscheduler failed");
    exit(-1);
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}