// ROS2 subscriber node with two callbacks to the /camera topic

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <list>
#include <thread>
#include <cactus_rt/tracing.h>

#include "rclcpp/experimental/fifo_sched.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

using FileSink = cactus_rt::tracing::FileSink;
using TraceAggregator = cactus_rt::tracing::TraceAggregator;
using ThreadTracer = cactus_rt::tracing::ThreadTracer;


class SensorNode : public rclcpp::Node
{
public:
  SensorNode()
  : Node("subscriber_node")
  {
    StartTracing("actuation_node", "actuation_node.json");
    trace_aggregator = nullptr;
    main_tracer = std::make_shared<ThreadTracer>("main_thread");
    logging_tracer = std::make_shared<ThreadTracer>("logging_thread");

    // Create a subscription on the "camera" topic to log the image
    subscription_logger_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera", 10, std::bind(&SensorNode::data_logger, this, std::placeholders::_1));

    // Create a subscription on the "camera" topic to detect an object
    subscription_object_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera", 10, std::bind(&SensorNode::object_detector, this, std::placeholders::_1));

    // Create a publisher on the "stop" topic
    publisher_ = this->create_publisher<std_msgs::msg::Empty>("stop", 10);

    // TODO: Omit this in the exercise
    sched_param sp;
    sp.sched_priority = HIGH;
    subscription_object_->sched_param(sp);
  }

  ~SensorNode() {
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

  void data_logger(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto span = logging_tracer->WithSpan("data_logger");

    // Print the image encoding
    RCLCPP_INFO(this->get_logger(), "Image encoding: %s", msg->encoding.c_str());

    // Print the image dimensions
    RCLCPP_INFO(this->get_logger(), "Image dimensions: %dx%d", msg->width, msg->height);
  }

  void object_detector(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto span = main_tracer->WithSpan("object_detector");
    
    // Detect an object in the image
    if (msg->data[0] == 1) {
      // Publish a message to /stop
      RCLCPP_INFO(this->get_logger(), "Object detected, stopping robot");

      // Create a new message
      auto msg = std::make_unique<std_msgs::msg::Empty>();

      // Publish the message
      publisher_->publish(std::move(msg));
    }
  }

  std::unique_ptr<TraceAggregator> trace_aggregator;
  std::shared_ptr<ThreadTracer> main_tracer;
  std::shared_ptr<ThreadTracer> logging_tracer;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_logger_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_object_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
};

int main() {
  // OMIT IN EXERCISE
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<SensorNode>();

  // Create an executor to put the node into
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Run the executor
  executor.spin();
  rclcpp::shutdown();
  return 0;
}