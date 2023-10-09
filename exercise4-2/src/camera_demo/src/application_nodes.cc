#include "application_nodes.h"
#include <cstdlib>
#include <chrono>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "utils.h"

using cactus_rt::tracing::ThreadTracer;
using camera_demo_interfaces::msg::FakeImage;

CameraProcessingNode::CameraProcessingNode(
  std::shared_ptr<ThreadTracer> tracer_object_detector,
  std::shared_ptr<ThreadTracer> tracer_data_logger
) : Node("obj_detect"), tracer_object_detector_(tracer_object_detector), tracer_data_logger_(tracer_data_logger) {
  realtime_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive
  );
  besteffort_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive
  );

  rclcpp::SubscriptionOptions data_logging_options;
  data_logging_options.callback_group = besteffort_group_;
  subscription_data_logger_ = this->create_subscription<FakeImage>(
    "/image",
    10,
    std::bind(&CameraProcessingNode::DataLoggerCallback, this, std::placeholders::_1),
    data_logging_options
  );

  rclcpp::SubscriptionOptions obj_det_options;
  obj_det_options.callback_group = realtime_group_;
  subscription_object_detector_ = this->create_subscription<FakeImage>(
    "/image",
    10,
    std::bind(&CameraProcessingNode::ObjectDetectorCallback, this, std::placeholders::_1),
    obj_det_options
  );

  publisher_ = this->create_publisher<std_msgs::msg::Int64>("/actuation", 10);

  // initialization of latency random variable
  srand((unsigned)time(NULL));
}

void CameraProcessingNode::ObjectDetectorCallback(const FakeImage::SharedPtr image) {
  // A hack to trace the message passing delay between publisher and this node
  auto now = cactus_rt::NowNs();
  tracer_object_detector_->StartSpan("MessageDelay", nullptr, image->published_at_monotonic_nanos);
  tracer_object_detector_->EndSpan(now);

  {
    auto span = tracer_object_detector_->WithSpan("ObjectDetect");

    // variable duration to serialize the data between [20ms,35ms]
    unsigned int data_logger_latency = 20000 + (rand() % 15001);
    WasteTime(std::chrono::microseconds(3000));

    // Send a signal to the downstream actuation node
    std_msgs::msg::Int64 msg;
    msg.data = image->published_at_monotonic_nanos;
    publisher_->publish(msg);
  }
}

void CameraProcessingNode::DataLoggerCallback(const FakeImage::SharedPtr image) {
  // A hack to trace the message passing delay between publisher and this node
  auto now = cactus_rt::NowNs();
  tracer_data_logger_->StartSpan("MessageDelay", nullptr, image->published_at_monotonic_nanos);
  tracer_data_logger_->EndSpan(now);

  {
    auto span = tracer_data_logger_->WithSpan("DataLogger");

    // Assume it takes 6ms to serialize the data which is all on the CPU
    WasteTime(std::chrono::microseconds(6000));

    // Assume it takes about 1ms to write the data where it is blocking but yielded to the CPU.
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }
}

rclcpp::CallbackGroup::SharedPtr CameraProcessingNode::get_realtime_cbg() {
  if (!realtime_group_) {
    realtime_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
    );
  }
  return realtime_group_;
}

rclcpp::CallbackGroup::SharedPtr CameraProcessingNode::get_besteffort_cbg() {
  if (!besteffort_group_) {
    besteffort_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
    );
  }
  return besteffort_group_;
}

ActuationNode::ActuationNode(
  std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer
) : Node("actuation"), tracer_(tracer) {
  subscription_ = this->create_subscription<std_msgs::msg::Int64>(
    "/actuation",
    10,
    std::bind(&ActuationNode::MessageCallback, this, std::placeholders::_1)
  );
}

void ActuationNode::MessageCallback(const std_msgs::msg::Int64::SharedPtr published_at_timestamp) {
  auto now = cactus_rt::NowNs();

  // A hack to show the end to end latency, from the moment it was published to
  // the moment it is received by this node.
  tracer_->StartSpan("EndToEndDelay", nullptr, published_at_timestamp->data);
  tracer_->EndSpan(now);

  {
    auto span = tracer_->WithSpan("Actuation");
    WasteTime(std::chrono::microseconds(150));
  }
}
