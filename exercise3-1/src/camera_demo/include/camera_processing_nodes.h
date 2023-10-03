#ifndef ROSCON_2023_REALTIME_WORKSHOP_CAMERA_PROCESSING_NODES_H_
#define ROSCON_2023_REALTIME_WORKSHOP_CAMERA_PROCESSING_NODES_H_

#include <cactus_rt/tracing.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#include "camera_demo_interfaces/msg/fake_image.hpp"

class RealTimeObjectDetectorNode : public rclcpp::Node {
  std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer_;

  rclcpp::Subscription<camera_demo_interfaces::msg::FakeImage>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr                      publisher_;

 public:
  explicit RealTimeObjectDetectorNode(std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer);

 private:
  void ImageCallback(const camera_demo_interfaces::msg::FakeImage::SharedPtr image);
};

class NonRealTimeDataLoggerNode : public rclcpp::Node {
  std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer_;

  rclcpp::Subscription<camera_demo_interfaces::msg::FakeImage>::SharedPtr subscription_;

 public:
  explicit NonRealTimeDataLoggerNode(std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer);

 private:
  void ImageCallback(const camera_demo_interfaces::msg::FakeImage::SharedPtr image);
};

#endif
