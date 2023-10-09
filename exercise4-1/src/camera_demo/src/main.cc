#include <rclcpp/rclcpp.hpp>

#include "application_nodes.h"
#include "system_nodes.h"
#include "tracing.h"

int main(int argc, char** argv) {
  // initialization of ROS and DDS middleware
  rclcpp::init(argc, argv);

  auto camera_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("camera");
  auto camera_node = std::make_shared<ImagePublisherNode>(camera_tracer, 30.0);

  auto actuation_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("actuation");
  auto actuation_node = std::make_shared<ActuationNode>(actuation_tracer);

  auto object_detector_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("object_detector_callback");
  auto data_logger_tracer = std::make_shared<cactus_rt::tracing::ThreadTracer>("data_logger_callback");
  auto camera_processing_node = std::make_shared<CameraProcessingNode>(object_detector_tracer, data_logger_tracer);

  StartTracing("camera_demo_4_1", "exercise4-1.perfetto");
  RegisterThreadTracer(camera_tracer);
  RegisterThreadTracer(actuation_tracer);
  RegisterThreadTracer(object_detector_tracer);
  RegisterThreadTracer(data_logger_tracer);

  rclcpp::executors::SingleThreadedExecutor executor;

  // Exercise 4-1 (1): remove camera_node from executor (remove next line)
  executor.add_node(camera_node);

  executor.add_node(camera_processing_node);
  executor.add_node(actuation_node);

  // Exercise 4-1 (2)
  // - Define SingleThreadedExecutor image_pub_executor (hint: see line 27);
  // - add camera_node to image_pub_executor. (hint: see line 32)

  std::thread thr;
  // Exercise 4-1: uncomment next section, set priority and call spin()
  /*
  thr = std::thread([&image_pub_executor] {
    sched_param sch;
    // Exercise 4-1 (3): set priority to 90
    if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
      throw std::runtime_error{std::string("failed to set scheduler: ") + std::strerror(errno)};
    }
    // Exercise 4-1 (4): call spin() function
  });
  */
  executor.spin();
  rclcpp::shutdown();
  StopTracing();

  // Exercise 4-1 (5): join the thread (uncomment next line)
  // thr.join();
  
  return 0;
}
