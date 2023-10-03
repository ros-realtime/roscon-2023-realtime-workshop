#include "camera_processing_nodes.h"

#include "utils.h"

using cactus_rt::tracing::ThreadTracer;
using camera_demo_interfaces::msg::FakeImage;

RealTimeObjectDetectorNode::RealTimeObjectDetectorNode(
  std::shared_ptr<ThreadTracer> tracer
) : Node("obj_detect"), tracer_(tracer) {
  subscription_ = this->create_subscription<FakeImage>(
    "/image",
    10,
    std::bind(&RealTimeObjectDetectorNode::ImageCallback, this, std::placeholders::_1)
  );

  publisher_ = this->create_publisher<std_msgs::msg::Int64>("/actuation", 10);
}

void RealTimeObjectDetectorNode::ImageCallback(const FakeImage::SharedPtr image) {
  // A hack to trace the message passing delay between publisher and this node
  auto now = cactus_rt::NowNs();
  tracer_->StartSpan("MessageDelay", nullptr, image->published_at_monotonic_nanos);
  tracer_->EndSpan(now);

  {
    auto span = tracer_->WithSpan("ObjectDetect");

    // Pretend it takes 4000 ms to do object detection.
    WasteTime(std::chrono::microseconds(4000));

    // Send a signal to the downstream actuation node
    std_msgs::msg::Int64 msg;
    msg.data = image->published_at_monotonic_nanos;
    publisher_->publish(msg);
  }
}

NonRealTimeDataLoggerNode::NonRealTimeDataLoggerNode(
  std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer
) : Node("data_logger"), tracer_(tracer) {
  subscription_ = this->create_subscription<FakeImage>(
    "/image",
    10,
    std::bind(&NonRealTimeDataLoggerNode::ImageCallback, this, std::placeholders::_1)
  );
}

void NonRealTimeDataLoggerNode::ImageCallback(const FakeImage::SharedPtr image) {
  auto span = tracer_->WithSpan("DataLogger");

  // Assume it takes 1ms to serialize the data which is all on the CPU
  WasteTime(std::chrono::microseconds(1000));

  // Assume it takes about 1ms to write the data where it is blocking but yielded to the CPU.
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
}
