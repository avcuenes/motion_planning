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

    path_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("path_points", 10);
    
}


void MP::obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{ 
  

  rrtStar.clearObstacles();
  for (size_t i = 0; i < msg->data.size(); i += 3) {
      double obstacleX = static_cast<double>(msg->data[i]);
      double obstacleY = static_cast<double>(msg->data[i + 1]);
      double obstacleR = static_cast<double>(msg->data[i + 2]);
      rrtStar.addObstacle(obstacleX, obstacleY,obstacleR);
      RCLCPP_INFO(get_logger(), "obstacle point: [%.2f, %.2f, %.2f]",
        obstacleX, obstacleY,obstacleR );
  }

    
}


void MP::homepoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{ 
  const auto home_point_sub = msg->data;
  homepoint.x = home_point_sub[0];
  homepoint.z = home_point_sub[2];
  
  RCLCPP_INFO(get_logger(), "homepoint: [%.2f, %.2f, %.2f]",
              homepoint.x, homepoint.y, homepoint.z);
  publishPath();
  
}


void MP::publishPath() {

    rrtStar.setHomeandTargetPoint(homepoint.x,homepoint.y,targetpoint.x,targetpoint.y);
    rrtStar.setMapConstraint(-1, -1, 11, 11);
    rrtStar.setStepSize(1);
    rrtStar.setMaxIterations(50);
    std::vector<NodeRRT*> nodes = rrtStar.RRTStarAlgorithm();
    // Assuming the goal is the last node in the vector
     
    std_msgs::msg::Float32MultiArray path_msg;
    path_msg.data.clear();

    // Extract x and y coordinates from the nodes
    for (const auto& node : nodes) {
        std::cout << nodes.size() << std::endl;
        RCLCPP_INFO(get_logger(), "homepoint: [%.2f, %.2f]",
              node->x, node->y);
        path_msg.data.push_back(static_cast<float>(node->x));
        path_msg.data.push_back(static_cast<float>(node->y));
    }

    // Publish the path coordinates
    path_publisher_->publish(path_msg);
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
