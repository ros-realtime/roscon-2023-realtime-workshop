#include "system_nodes.h"

#include <cactus_rt/utils.h>

#include <string>

#include "utils.h"

using camera_demo_interfaces::msg::FakeImage;

ImagePublisherNode::ImagePublisherNode(
  std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer,
  double frequency_hz
) : Node("imagepub"), tracer_(tracer) {
  timer_ = this->create_wall_timer(
    std::chrono::microseconds(static_cast<int>(1000000 / frequency_hz)),
    std::bind(&ImagePublisherNode::TimerCallback, this)
  );

  publisher_ = this->create_publisher<FakeImage>("/image", 10);
  last_image_publisher_timepoint = 0;
}

void ImagePublisherNode::TimerCallback() {
  FakeImage img;
  img.data = {127};
  img.published_at_monotonic_nanos = cactus_rt::NowNs();

  publisher_->publish(img);

  // show publisher period
  // omit first publisher call
  if (last_image_publisher_timepoint != 0){
    tracer_->StartSpan("period", nullptr, last_image_publisher_timepoint);
    tracer_->EndSpan(img.published_at_monotonic_nanos);
    {
      auto span = tracer_->WithSpan("ImagePublisher");
    }
  }

  last_image_publisher_timepoint = img.published_at_monotonic_nanos;

}
