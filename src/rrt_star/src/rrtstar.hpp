#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <iostream>
using std::placeholders::_1;

class MP : public rclcpp::Node
{
public:
  MP();

private:
    /*
    Define structure for home point and target point */
    
    struct {
        float x;
        float y;
        float z;
    } homepoint,targetpoint;

    /*Define structure for obstacle point  */
    struct {
        float x;
        float y;
        float r;
    } obstaclepoint;

                

    /* obstacles location callback function*/
    void obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) ;
    /*homepoint location callback function*/
    void homepoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) ;
    /*targetpoint location callback function*/
    void targetpoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) ;
    

    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstaclepoints_;    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr homepoint_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr targetpoint_;
    
};
