#include "system_nodes.h"

#include <cactus_rt/utils.h>

#include <string>

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

std::thread                                                thr;
std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
ImagePublisherNode::SharedPtr                              node;

void StartImagePublisherNode() {
  // Has to be an unique pointer as we need to initialize it after rclcpp::init called in main.
  executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

  node = std::make_shared<ImagePublisherNode>(60.0);
  executor->add_node(node);

  thr = std::thread([] {
    sched_param sch;
    sch.sched_priority = 90;
    if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
      throw std::runtime_error{std::string("failed to set scheduler: ") + std::strerror(errno)};
    }

    executor->spin();
  });
}

void JoinImagePublisherNode() {
  thr.join();
  executor = nullptr;  // Delete the executor.
  node = nullptr;      // Delete the node, otherwise on shutdown there will be a segfault.
}
