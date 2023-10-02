#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <list>
#include <thread>
#include <cactus_rt/tracing.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

using ThreadTracer = cactus_rt::tracing::ThreadTracer;


class SensorNode : public rclcpp::Node
{
public:
  SensorNode();
  ~SensorNode();
private:
  void data_logger(const sensor_msgs::msg::Image::SharedPtr msg);
  void object_detector(const sensor_msgs::msg::Image::SharedPtr msg);
  std::shared_ptr<ThreadTracer> objdet_tracer_;
  std::shared_ptr<ThreadTracer> logging_tracer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_logger_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_object_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
};