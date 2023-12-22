#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <iostream>
#include "rrtstar.cpp"

enum Algorithms 
{   KinoRRTStar = 0, 
    AStar ,
    CHOMP 
};
using std::placeholders::_1;

class MP : public rclcpp::Node
{
public:
  MP();

  int algorithm{0};

private:
    /*
    Define structure for home point and target poi"nt */
    
    struct {
        float x;
        float y;
        float z;
    } homepoint,targetpoint;

    /*
    Define structure for map constraint */
    
    struct {
        double xmin,ymin;
        double xmax,ymax;
    } mapconstraint;

    

    /*Define structure for obstacle point  */
    struct {
        float x;
        float y;
        float r;
    } obstaclepoint;

    RRTStar rrtStar;

   
    

    /* obstacles location callback function*/
    void obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) ;
    /*homepoint location callback function*/
    void homepoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) ;
    /*targetpoint location callback function*/
    void targetpoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) ;
    /*map callback function*/
    void map_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) ;

    // Publish the path coordinates
    void    publishPath();    
    

    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstaclepoints_;    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr homepoint_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr targetpoint_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mapconst_;
    

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr path_publisher_;

    bool message_obstacle_;
    bool message_homepoint_;
    bool message_targetpoint_;
    bool message_mapconst_;

    
};
