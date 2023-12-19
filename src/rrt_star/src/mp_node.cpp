#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <iostream>
#include "mp_node.hpp"
using std::placeholders::_1;

MP::MP() :  Node("MotionPlanning") 
{
    obstaclepoints_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "obstacle_point", 10, std::bind(&MP::obstacle_callback, this, _1));
  
    homepoint_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "home_point", 10, std::bind(&MP::homepoint_callback, this, _1));
    
    targetpoint_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "target_point", 10, std::bind(&MP::targetpoint_callback, this, _1));
    
}


void MP::obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{ 
  const auto obstacle_point_sub = msg->data;

    
    RCLCPP_INFO(get_logger(), "obstacle point: [%.2f, %.2f, %.2f]",
              obstacle_point_sub[0], obstacle_point_sub[1], obstacle_point_sub[2]);
}


void MP::homepoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{ 
  const auto home_point_sub = msg->data;
  homepoint.x = home_point_sub[0];
  homepoint.y = home_point_sub[1];
  homepoint.z = home_point_sub[2];
  
    RCLCPP_INFO(get_logger(), "homepoint: [%.2f, %.2f, %.2f]",
              homepoint.x, homepoint.y, homepoint.z);
}


void MP::targetpoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{ 
  const auto target_point_sub = msg->data;
  targetpoint.x = target_point_sub[0];
  targetpoint.y = target_point_sub[1];
  targetpoint.z = target_point_sub[2];
  
  RCLCPP_INFO(get_logger(), "target_point: [%.2f, %.2f, %.2f]",
              targetpoint.x,  targetpoint.y,  targetpoint.z);
}



int main(int argc, char * argv[])
{ 
  std::cout << "enes "<< std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MP>());
  rclcpp::shutdown();
  return 0;
}
