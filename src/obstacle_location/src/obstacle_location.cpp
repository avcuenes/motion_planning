#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;


class ObstaclePointPublisher : public rclcpp::Node
{
public:
  ObstaclePointPublisher()
  : Node("obstacle_point_publisher"), count_(0)
  {
    publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("obstacle_point", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ObstaclePointPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float32MultiArray();
    // Obstacle point location published by form x,y  coordinates and radius of obstacle
    message.data = {1.0,4.5,1.2, 1.2,2.4,2, 5.3,2.5,2, 8.2,5,3};

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
  rclcpp::spin(std::make_shared<ObstaclePointPublisher>());
  rclcpp::shutdown();
  return 0;
}
