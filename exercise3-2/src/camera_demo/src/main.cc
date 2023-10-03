#include <rclcpp/rclcpp.hpp>

#include "application_nodes.h"
#include "system_nodes.h"
#include "tracing.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  StartImagePublisherNode();

  auto actuation_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("actuation");
  auto actuation_node = std::make_shared<ActuationNode>(actuation_tracer);

  auto camera_processing_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("camera_processing");
  auto camera_processing_node = std::make_shared<CameraProcessingNode>(camera_processing_tracer);

  StartTracing("camera_demo_3_1", "exercise3-1.perfetto");
  RegisterThreadTracer(actuation_tracer);
  RegisterThreadTracer(camera_processing_tracer);

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(camera_processing_node);
  executor.add_node(actuation_node);
  executor.spin();

  rclcpp::shutdown();
  StopTracing();

  JoinImagePublisherNode();
  return 0;
}
