#include <rclcpp/rclcpp.hpp>

#include "application_nodes.h"
#include "system_nodes.h"
#include "tracing.h"

int main(int argc, char** argv) {
  // initialization of ROS and DDS middleware
  rclcpp::init(argc, argv);

  StartImagePublisherNode();

  auto actuation_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("actuation");
  auto actuation_node = std::make_shared<ActuationNode>(actuation_tracer);

  auto object_detector_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("object_detector_callback");
  auto data_logger_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("data_logger_callback");
  auto camera_processing_node = std::make_shared<CameraProcessingNode>(object_detector_tracer, data_logger_tracer);
  auto data_logger_group = camera_processing_node->get_besteffort_cbg();
  auto object_detector_group = camera_processing_node->get_realtime_cbg();

  StartTracing("camera_demo_4_2", "exercise4-2.perfetto");
  RegisterThreadTracer(actuation_tracer);
  RegisterThreadTracer(object_detector_tracer);
  RegisterThreadTracer(data_logger_tracer);

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(camera_processing_node);
  executor.add_node(actuation_node);
  executor.spin();

  rclcpp::shutdown();
  StopTracing();

  JoinImagePublisherNode();
  return 0;
}
