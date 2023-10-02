#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cactus_rt/tracing.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

using ThreadTracer = cactus_rt::tracing::ThreadTracer;


class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode();
  ~PublisherNode();
private:
  void publish();
  std::shared_ptr<ThreadTracer> publishing_tracer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};