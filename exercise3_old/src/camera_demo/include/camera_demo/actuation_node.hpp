#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cactus_rt/tracing.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

using ThreadTracer = cactus_rt::tracing::ThreadTracer;


class ActuationNode : public rclcpp::Node
{
public:
  ActuationNode();
  ~ActuationNode();
private:
  void stop_robot(const std_msgs::msg::Empty::SharedPtr msg);
  std::shared_ptr<ThreadTracer> actuation_tracer_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
};

