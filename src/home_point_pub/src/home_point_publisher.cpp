#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;


class HomePointPublisher : public rclcpp::Node
{
public:
  HomePointPublisher()
  : Node("home_point_publisher"), count_(0)
  {
    publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("home_point", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&HomePointPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float32MultiArray();
    // Home point location published by form x,y,z coordinates
    message.data = {0.0, 0.0, 3.0};

    // Publish the message
    publisher_->publish(message);

    RCLCPP_INFO(get_logger(), "Published: [%.2f, %.2f, %.2f]",
                message.data[0], message.data[1], message.data[2]);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HomePointPublisher>());
  rclcpp::shutdown();
  return 0;
}
