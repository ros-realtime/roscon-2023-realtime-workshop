#include <rclcpp/rclcpp.hpp>

#include "application_nodes.h"
#include "system_nodes.h"
#include "tracing.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  StartImagePublisherNode();

  auto actuation_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("actuation");
  auto actuation_node = std::make_shared<ActuationNode>(actuation_tracer);

  auto object_detector_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("object_detector_callback");
  auto data_logger_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("data_logger_callback");
  auto camera_processing_node = std::make_shared<CameraProcessingNode>(object_detector_tracer, data_logger_tracer);

  StartTracing("camera_demo_3_1", "exercise3-1.perfetto");
  RegisterThreadTracer(actuation_tracer);
  RegisterThreadTracer(object_detector_tracer);
  RegisterThreadTracer(data_logger_tracer);

  // TODO: Create executor, add nodes to it, and call spin() on executor

  rclcpp::shutdown();
  StopTracing();

  JoinImagePublisherNode();
  return 0;
}
