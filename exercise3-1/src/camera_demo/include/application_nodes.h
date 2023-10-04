#ifndef ROSCON_2023_REALTIME_WORKSHOP_CAMERA_PROCESSING_NODES_H_
#define ROSCON_2023_REALTIME_WORKSHOP_CAMERA_PROCESSING_NODES_H_

#include <cactus_rt/tracing.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#include "camera_demo_interfaces/msg/fake_image.hpp"

class CameraProcessingNode : public rclcpp::Node {
  std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer_object_detector_;
  std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer_data_logger_;

  rclcpp::Subscription<camera_demo_interfaces::msg::FakeImage>::SharedPtr subscription_object_detector_;
  rclcpp::Subscription<camera_demo_interfaces::msg::FakeImage>::SharedPtr subscription_data_logger_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr                      publisher_;

 public:
  CameraProcessingNode(
    std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer_object_detector,
    std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer_data_logger
  );

 private:
  void ObjectDetectorCallback(const camera_demo_interfaces::msg::FakeImage::SharedPtr image);
  void DataLoggerCallback(const camera_demo_interfaces::msg::FakeImage::SharedPtr image);
};

class ActuationNode : public rclcpp::Node {
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;

  std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer_;

 public:
  explicit ActuationNode(std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer);

 private:
  void MessageCallback(const std_msgs::msg::Int64::SharedPtr published_at_timestamp);
};

#endif
