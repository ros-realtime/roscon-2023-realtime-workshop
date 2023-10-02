#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cactus_rt/tracing.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"

#include "camera_demo/publisher_node.hpp"
#include "camera_demo/sensor_node.hpp"
#include "camera_demo/actuation_node.hpp"

int main() {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<SensorNode>();

  // Put each sensor node subscriber in two different callback groups
  rclcpp::CallbackGroup::SharedPtr data_logger_group = node->get_besteffort_cbg();
  rclcpp::CallbackGroup::SharedPtr object_detector_group = node->get_realtime_cbg();  // OMIT IN EXERCISE

  // Create nodes for publisher and actuation nodes
  auto publisher_node = std::make_shared<PublisherNode>();
  auto actuation_node = std::make_shared<ActuationNode>();

  // Create an executor to put the callback groups in
  rclcpp::executors::SingleThreadedExecutor data_logging_executor;
  rclcpp::executors::SingleThreadedExecutor real_time_executor;  // OMIT IN EXERCISE
  data_logging_executor.add_callback_group(data_logger_group, node->get_node_base_interface());
  real_time_executor.add_callback_group(object_detector_group, node->get_node_base_interface());  // OMIT IN EXERCISE
  real_time_executor.add_node(publisher_node);
  real_time_executor.add_node(actuation_node);

  // OMIT IN EXERCISE
  // Create thread for real-time executor
  std::thread real_time_thread([&real_time_executor]() {
    // Give this thread real-time priority
    sched_param sch;
    sch.sched_priority = 50;
    if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
      perror("sched_setscheduler failed");
      exit(-1);
    }
    real_time_executor.spin();
  });

  // OMIT IN EXERCISE
  // Run the data logger in the main thread, without realtime priority
  data_logging_executor.spin();
  return 0;
}