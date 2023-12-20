
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style


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
        
        self.path_callback = self.create_subscription(
            Float32MultiArray,
            "path_points",
            self.path_callback,
            10)

        
        
        self.homepoint  # prevent unused variable warning
        self.obstaclepoints  # prevent unused variable warning
        self.targetpoints  # prevent unused variable warning
        
        self.homepointvector = np.zeros(3)
        self.obstaclepointsvector = 0.0
        self.targetpointsvector = np.zeros(3)
        
        self.obs_x = [1]
        self.obs_y = [1]
        self.obs_z = [1]
        
        self.points_data = [(1, 2), (3, 4), (5, 6), (7, 8)]
        self.path_data = [(1, 2), (3, 4), (5, 6), (7, 8)]  # Replace with your actual path data

    
    def visualize_obstacles(self):
            
        for x, y, radius in zip(self.obs_x, self.obs_y, self.obs_z):
            circle = plt.Circle((x, y), radius, color='red', fill=True, label='Obstacle')
            plt.gca().add_patch(circle)
            
    
    def visualize_points_and_path(self,points, pathx,pathy):
        x_values = [point[0] for point in points]
        y_values = [point[1] for point in points]

        plt.scatter(x_values, y_values, color='blue', label='Points')
        self.visualize_obstacles()

        
        plt.scatter(pathx, pathy, marker='o', linestyle='-', color='black')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Visualization of Points, Path, and Obstacles')
        plt.legend()
        plt.grid(True)
        plt.show()

    def homepoint_callback(self, msg):
        self.homepointvector[0] = msg.data[0]
        self.homepointvector[1] = msg.data[1]
        self.homepointvector[2] = msg.data[2]
    
    def targetpoint_callback(self, msg):
        self.targetpointsvector[0] = msg.data[0]
        self.targetpointsvector[1] = msg.data[1]
        self.targetpointsvector[2] = msg.data[2]
    
    def obstacles_callback(self, msg):
        self.obs_x = []
        self.obs_y = []
        self.obs_z = []
        
        for i in range(0, len(msg.data), 3):
            self.obs_x.append(float(msg.data[i]))
            self.obs_y.append(float(msg.data[i+1]))
            self.obs_z.append(float(msg.data[i+2]))
           
        
        print(self.obs_x, self.obs_y, self.obs_z)
     
    
    def path_callback(self,msg):
        self.path_x = []
        self.path_y = []
        for i in range(0, len(msg.data), 2):
            self.path_x.append(float(msg.data[i]))
            self.path_y.append(float(msg.data[i+1]))
        
        
        self.points_data = [(self.homepointvector[0],self.homepointvector[1]),(self.targetpointsvector[0],self.targetpointsvector[1])]
        self.visualize_points_and_path(self.points_data, self.path_x,self.path_y)
        
        


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
