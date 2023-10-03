#include "system_nodes.h"

#include <cactus_rt/utils.h>

#include "utils.h"

using camera_demo_interfaces::msg::FakeImage;

ImagePublisherNode::ImagePublisherNode(
  double frequency_hz
) : Node("imagepub") {
  timer_ = this->create_wall_timer(
    std::chrono::microseconds(static_cast<int>(1000000 / frequency_hz)),
    std::bind(&ImagePublisherNode::TimerCallback, this)
  );

  publisher_ = this->create_publisher<FakeImage>("/image", 10);
}

void ImagePublisherNode::TimerCallback() {
  FakeImage img;
  img.data = {127};
  img.published_at_monotonic_nanos = cactus_rt::NowNs();

  publisher_->publish(img);
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
  tracer_->StartSpan("MessageProcessing", nullptr, published_at_timestamp->data);
  tracer_->EndSpan(now);

  {
    auto span = tracer_->WithSpan("Actuation");
    WasteTime(std::chrono::microseconds(150));
  }
}
