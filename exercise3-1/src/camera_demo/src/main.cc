#include <rclcpp/rclcpp.hpp>

#include "camera_processing_nodes.h"
#include "system_nodes.h"
#include "tracing.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto image_publisher_node = std::make_shared<ImagePublisherNode>(60.0);

  auto actuation_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("actuation");
  auto actuation_node = std::make_shared<ActuationNode>(actuation_tracer);

  auto object_detector_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("obj_detect");
  auto object_detector_node = std::make_shared<RealTimeObjectDetectorNode>(object_detector_tracer);

  auto data_logger_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("data_logger");
  auto data_logger_node = std::make_shared<NonRealTimeDataLoggerNode>(data_logger_tracer);

  StartTracing("camera_demo_3_1", "exercise3-1.perfetto");
  RegisterThreadTracer(actuation_tracer);
  RegisterThreadTracer(object_detector_tracer);
  RegisterThreadTracer(data_logger_tracer);

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(image_publisher_node);
  executor.add_node(object_detector_node);
  executor.add_node(data_logger_node);
  executor.add_node(actuation_node);
  executor.spin();

  rclcpp::shutdown();

  StopTracing();
  return 0;
}
