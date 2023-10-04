#include <rclcpp/rclcpp.hpp>

#include "application_nodes.h"
#include "system_nodes.h"
#include "tracing.h"

int main(int argc, char** argv) {

  // Set real-time priority for all middleware threads
  // (assignment of scheduling priority to main thread is before `rclcpp::init()`)
  // ... which also runs the executor in this main thread with the real-time priority. (see below)
  // TODO
  // reduce then the priority of best-effort thread below
  //  sched_param sch;
  // sch.sched_priority = 50;
  // if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
  //   perror("sched_setscheduler failed");
  //  exit(-1);
  // }

  // initialization of ROS and DDS middleware
  rclcpp::init(argc, argv);

  StartImagePublisherNode();

  auto actuation_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("actuation");
  auto actuation_node = std::make_shared<ActuationNode>(actuation_tracer);

  auto object_detector_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("object_detector_callback");
  auto data_logger_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("data_logger_callback");
  auto camera_processing_node = std::make_shared<CameraProcessingNode>(object_detector_tracer, data_logger_tracer);

  StartTracing("camera_demo_4_1", "exercise4-1.perfetto");
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
