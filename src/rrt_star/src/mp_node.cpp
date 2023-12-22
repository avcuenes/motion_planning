#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <iostream>
#include "mp_node.hpp"
using std::placeholders::_1;

MP::MP() :  Node("MotionPlanning") 
{   
    // MP class constructor

    // Define topics for subscribers
    obstaclepoints_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "obstacle_point", 10, std::bind(&MP::obstacle_callback, this, _1));
  
    homepoint_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "home_point", 10, std::bind(&MP::homepoint_callback, this, _1));
    
    targetpoint_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "target_point", 10, std::bind(&MP::targetpoint_callback, this, _1));

    mapconst_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "map_const", 10, std::bind(&MP::map_callback, this, _1));

      
    // Define the topics for publisher
    path_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("path_points", 10);

    // Set messages flags 
    message_obstacle_ = false;
    message_homepoint_ = false;
    message_targetpoint_ = false;
    message_mapconst_ = false;

    
}
// map callback function
void MP::map_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{ 
  const auto map_sub = msg->data;
  mapconstraint.xmin = map_sub[0];
  mapconstraint.ymin = map_sub[1];
  mapconstraint.xmax = map_sub[2];
  mapconstraint.ymax = map_sub[3];
  
  message_mapconst_ = true;
  
  RCLCPP_INFO(get_logger(), "map constraints: [%.2f, %.2f, %.2f,%.2f]",
              mapconstraint.xmin, mapconstraint.ymin, mapconstraint.xmax, mapconstraint.ymax);
  
}

// obstacle callback function
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

  message_obstacle_ = true;
}

// homepoint callback function
void MP::homepoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{ 
  const auto home_point_sub = msg->data;
  homepoint.x = home_point_sub[0];
  homepoint.z = home_point_sub[2];
  
  RCLCPP_INFO(get_logger(), "homepoint: [%.2f, %.2f, %.2f]",
              homepoint.x, homepoint.y, homepoint.z);
  
  message_homepoint_ = true;
  
  if(message_homepoint_ && message_mapconst_ && message_targetpoint_  )
  {
    publishPath();
  }
  
  
}

// publish function for path 
void MP::publishPath() {

    std_msgs::msg::Float32MultiArray path_msg;
    path_msg.data.clear();
    switch (algorithm) {
		case Algorithms::KinoRRTStar:{
      rrtStar.setHomeandTargetPoint(homepoint.x,homepoint.y,targetpoint.x,targetpoint.y);
      rrtStar.setMapConstraint(mapconstraint.xmin, mapconstraint.ymin, mapconstraint.xmax, mapconstraint.ymax);
      rrtStar.setStepSize(1);
      rrtStar.setMaxIterations(50);
      std::vector<NodeRRT*> nodes = rrtStar.RRTStarAlgorithm();
     
      // Assuming the goal is the last node in the vector
      NodeRRT* current = nodes.back();

      while (current != nullptr) {
          path_msg.data.push_back(static_cast<float>(current->x));
          path_msg.data.push_back(static_cast<float>(current->y));
          current = current->parent;
      }
      
      }
			break;

		case Algorithms::AStar: {
				/* code */
			}
			break;

		case Algorithms::CHOMP: {
			}
			break;
		}

    // Publish the path coordinates
    path_publisher_->publish(path_msg);
}


// target point callback function
void MP::targetpoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{ 
  const auto target_point_sub = msg->data;
  targetpoint.x = target_point_sub[0];
  targetpoint.y = target_point_sub[1];
  targetpoint.z = target_point_sub[2];
  
  RCLCPP_INFO(get_logger(), "target_point: [%.2f, %.2f, %.2f]",
              targetpoint.x,  targetpoint.y,  targetpoint.z);

  message_targetpoint_ = true;
}



int main(int argc, char * argv[])
{ 
  std::cout << "MotionPlanning"<< std::endl;
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MP>());
  rclcpp::shutdown();
  return 0;
}
