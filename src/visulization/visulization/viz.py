
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('viz')
        self.homepoint = self.create_subscription(
            Float32MultiArray,
            'home_point',
            self.homepoint_callback,
            10)
        
        self.obstaclepoints = self.create_subscription(
            Float32MultiArray,
            'obstacle_point',
            self.obstacles_callback,
            10)
        
        self.targetpoints = self.create_subscription(
            Float32MultiArray,
            'target_point',
            self.targetpoint_callback,
            10)
        
        
        self.homepoint  # prevent unused variable warning
        self.obstaclepoints  # prevent unused variable warning
        self.targetpoints  # prevent unused variable warning
        
        self.homepointvector = np.zeros(3)
        self.obstaclepointsvector = 0.0
        self.targetpointsvector = np.zeros(3)
    

    def homepoint_callback(self, msg):
        print("home",msg.data)
        self.homepointvector[0] = msg.data[0]
        self.homepointvector[1] = msg.data[1]
        self.homepointvector[2] = msg.data[2]
    
    def targetpoint_callback(self, msg):
        print("target",msg.data)
        self.targetpointsvector[0] = msg.data[0]
        self.targetpointsvector[1] = msg.data[1]
        self.targetpointsvector[2] = msg.data[2]
    
    def obstacles_callback(self, msg):
        self.obstaclepointsvector = np.zeros(len(msg.data))
        print(msg.data)
        for i in range(0,len(msg.data)):
            self.obstaclepointsvector[i] = msg.data[i]
            print("obstacle",msg.data[i])
    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
