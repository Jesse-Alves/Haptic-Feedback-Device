#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import pi, sqrt
from numpy import arccos, arctan2
import numpy as np
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray

# ================================== Models
#Dimentions
a1 = 0.093
a2 = 0.100
a3 = 0.100
a4 = 0.093
a5 = 0.050

# =======> Trajectory Import via csv file
traj_maze1_HighCost = np.genfromtxt('/home/telecom/haptic_ws/src/haptic_nodes/scripts/traj_maze1_HighCost.csv', delimiter=',')
traj_maze1_HighCost = 10*traj_maze1_HighCost

'''
a1 = 0.063
a2 = 0.074
a3 = 0.075
a4 = 0.063
a5 = 0.025'''

# Work 10 times in the RVIZ.
a1 = a1*10
a2 = a2*10
a3 = a3*10
a4 = a4*10
a5 = a5*10

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')

        # Creating the Publishers
        self.publisher_ = self.create_publisher(JointTrajectory, '/haptic_trajectory_controller/joint_trajectory', 1)
        self.positionEE_publisher = self.create_publisher(Float64MultiArray,'/pose_end_effector', 1)
        self.encoder_angles_publisher = self.create_publisher(Float64MultiArray,'/encoder_angles',1)        
       
        # Elipse Trajectory ===========================================
        self.x_initial = -0.25
        self.y_initial = 1.0

        # Loop in ROS
        self.counter = 0
        self.timer_period = 0.04      # seconds
        self.t = 0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
    def fkm(self, q1, q5):
        # Coordinates of P2 with respect to P1
        x2 = a1 * np.cos(q1)
        y2 = a1 * np.sin(q1)
        P2 = np.array([x2, y2])
        
        # Coordinates of P4 with respect to P1
        x4 = a4 * np.cos(q5) - a5
        y4 = a4 * np.sin(q5)
        P4 = np.array([x4, y4])
        
        # FKM
        P2Ph = (a2 * 2 - a3 * 2 + np.linalg.norm(P4 - P2) ** 2) / (2 * np.linalg.norm(P4 - P2))
        Ph = P2 + (P2Ph / np.linalg.norm(P2 - P4)) * (P4 - P2)
        P3Ph = np.sqrt(a2 * 2 - P2Ph * 2)
        
        # Coordinates of P3 with respect to P1
        x3 = Ph[0] + (P3Ph / np.linalg.norm(P2 - P4)) * (P4[1] - P2[1])
        y3 = Ph[1] - (P3Ph / np.linalg.norm(P2 - P4)) * (P4[0] - P2[0])
        
        #P3 = np.array([x3, y3])

        return x3, y3

    def ikm(self, px, py):
        
        a13 = sqrt(px**2 + py**2)
        a53 = sqrt(((px+a5)**2)+py**2)

        alpha1 = arccos(((a1**2)+(a13**2)-(a2**2))/(2*a1*a13))
        beta1 = arctan2(py,-px) 
        theta1 = pi - alpha1 - beta1

        alpha5 = arctan2(py,px+a5)
        beta5 = arccos(((a53**2)+(a4**2)-(a3**2))/(2*a53*a4))
        theta5 = alpha5 + beta5

        # Finding the angles theta2 and theta3 for the simulation
        beta2 = arccos(((a2**2)+(a13**2)-(a1**2))/(2*a2*a13))
        beta3 = arccos(((a3**2)+(a53**2)-(a4**2))/(2*a3*a53))
        beta4 = pi - beta3 - beta5

        alpha2 = pi - alpha1 - beta2
        alpha3 = pi - alpha5 - beta1

        theta2 = pi - alpha2
        theta3 = pi - beta2 - alpha3 - beta3
        theta4 = -(pi - beta4)

        return theta1, theta2, theta3, theta4, theta5

    def timer_callback(self):
        # Set the objects
        self.msg = JointTrajectory()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.joint_names = ['joint1','joint2','joint3','joint4','joint5']        

        # Creating a ELIPSE trajectory ================================
        px_trajectory = 0.7*math.cos(0.5*self.t) + self.x_initial
        py_trajectory = 0.35*math.sin(0.5*self.t) + self.y_initial
        #==============================================================

        # Initial Position
        #px_trajectory = -0.25
        #py_trajectory = 1.0

        # Creating a trajectory via csv file ================================
        if self.counter == len(traj_maze1_HighCost):
            self.counter = 0

        px_trajectory = traj_maze1_HighCost[self.counter][0]
        py_trajectory = traj_maze1_HighCost[self.counter][1]
        self.counter = self.counter + 1
        #====================================================================
        

        # Pseudo Movement ==============================================================
        theta1, theta2, theta3, theta4, theta5 = self.ikm(px_trajectory, py_trajectory)
        self.xe, self.ye = self.fkm(theta1,theta5)
        theta1, theta2, theta3, theta4, theta5 = self.ikm(self.xe, self.ye)
        # ==============================================================================       

        # Move the robot ===============================================================
        self.point = JointTrajectoryPoint()
        self.point.positions = [theta1,theta2,theta3,theta4,theta5]
        self.point.time_from_start.sec = 1
        self.msg.points = [self.point]
        self.publisher_.publish(self.msg)
        # ==============================================================================

        # Publish the EE position and Pseudo Encoder Angles ============================
        
        #Publishing the EE Position
        EEpose_msg = Float64MultiArray()
        EEpose_msg.data = [self.xe, self.ye]
        self.positionEE_publisher.publish(EEpose_msg)

        # Publish the Pseudo Encoder Angles
        Encoder_angles = Float64MultiArray()
        Encoder_angles.data = [theta1, theta5, self.xe, self.ye]
        self.encoder_angles_publisher.publish(Encoder_angles)
        # ==============================================================================

        # Time of iteration
        self.t += self.timer_period

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