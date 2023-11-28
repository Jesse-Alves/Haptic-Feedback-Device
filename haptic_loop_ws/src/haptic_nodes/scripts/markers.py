#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray

from math import cos, sin, pi, sqrt
from numpy import arccos, arctan, arctan2, transpose, array
from numpy.linalg import norm, inv
import numpy as np
import math
import time
import csv

'''
rospy.init_node('csv_reader')

csv_file_path = 'sphere_center.csv'

with open(self.csv_file_path, 'r') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        spheres_info, x, y= row
'''

# Spheres
spheres_info = np.genfromtxt('/home/telecom/haptic_ws/src/haptic_nodes/scripts/maze1_HighCost.csv', delimiter=',')
#spheres_info = (array([[0.1,0.08,0.69],[0.1,-0.36,0.66],[0.1,-0.51,0.69],[0.1,-0.65,0.70],[0.1,0.35,0.36]]))/10


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')

        #self.get_logger().info(f'THE SPHERES INFO {spheres_info[1]},   {spheres_info[1][1]}')

        self.publisher_marker_ = self.create_publisher(Marker, '/reference_marker', 1)
        #self.maze_publisher = self.create_publisher(Float64MultiArray, '/maze', 1)

        self.timer_period = 0.04  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.t = 0
        self.send = True

    def timer_callback(self):
    
        #maze_msg = Float64MultiArray()
        #maze_msg.data = [self.xe, self.ye]
        #self.maze_publisher.publish(maze_msg)

        for k in range(len(spheres_info)):
            
            #self.get_logger().info(spheres_info, x[k], y[k])

            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f'sphere{k}'
            marker.header.frame_id = "world"
            marker.id=k
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = 10*spheres_info[k][1]
            marker.pose.position.y = 10*spheres_info[k][2]
            marker.pose.position.z = 0.12
            marker.scale.x = 10*spheres_info[k][0]
            marker.scale.y = 10*spheres_info[k][0]
            marker.scale.z = 10*spheres_info[k][0]
            marker.color.a = 1.0
            marker.color.g = 1.0

            time.sleep(1.0)
            self.publisher_marker_.publish(marker)      


def main(args=None):
    rclpy.init(args=args)

    trajectory_publisher = TrajectoryPublisher()

    rclpy.spin(trajectory_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()