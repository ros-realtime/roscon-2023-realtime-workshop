// Listen to the /stop topic and stop the robot when a message is received

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;


class ActuationNode : public rclcpp::Node
{
public:
  ActuationNode()
  : Node("actuation_node")
  {
    // Create a subscription on the "stop" topic
    subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "stop", 10, std::bind(&ActuationNode::stop_robot, this, std::placeholders::_1));
  }

private:
  void stop_robot(const std_msgs::msg::Empty::SharedPtr)
  {
    // Stop the robot
    RCLCPP_INFO(this->get_logger(), "STOPPING ROBOT");
    bool stop = true;
  }

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
  bool stop = false;
};

int main() {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<ActuationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}