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
  auto data_logger_group = camera_processing_node->get_besteffort_cbg();
  auto object_detector_group = camera_processing_node->get_realtime_cbg();

  StartTracing("camera_demo_4_2", "exercise4-2.perfetto");
  RegisterThreadTracer(actuation_tracer);
  RegisterThreadTracer(camera_processing_tracer);

  rclcpp::executors::SingleThreadedExecutor real_time_executor;
  rclcpp::executors::SingleThreadedExecutor best_effort_executor;

  best_effort_executor.add_callback_group(data_logger_group, camera_processing_node->get_node_base_interface());
  real_time_executor.add_callback_group(object_detector_group, camera_processing_node->get_node_base_interface());
  real_time_executor.add_node(actuation_node);
  
  // Launch best effort thread
  std::thread best_effort_thread([&best_effort_executor]() {
    best_effort_executor.spin();
  });

  // Set the priority of this realtime thread
  sched_param sch;
  sch.sched_priority = 50;
  if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
    perror("sched_setscheduler failed");
    exit(-1);
  }
  real_time_executor.spin();

  rclcpp::shutdown();
  StopTracing();

  JoinImagePublisherNode();
  return 0;
}
