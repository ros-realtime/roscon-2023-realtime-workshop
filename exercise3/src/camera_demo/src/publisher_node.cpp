// ROS2 Node that publishes a camera image at 30Hz

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;


class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode()
  : Node("publisher_node")
  {
    // Create a publisher on the "camera" topic
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);

    // Create a timer that calls the publish function every 33ms
    timer_ = this->create_wall_timer(33ms, std::bind(&PublisherNode::publish, this));
  }

private:
  void publish()
  {
    // Create a new image message
    auto msg = std::make_unique<sensor_msgs::msg::Image>();

    // Fill the message header
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera";

    // Set the image dimensions
    msg->height = 480;
    msg->width = 640;

    // Set the image encoding
    msg->encoding = "rgb8";

    // Set the image data
    msg->data.resize(msg->height * msg->width * 3);

    // Publish the message
    publisher_->publish(std::move(msg));
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main() {
  // OMIT Everything below during exercise
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<PublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}