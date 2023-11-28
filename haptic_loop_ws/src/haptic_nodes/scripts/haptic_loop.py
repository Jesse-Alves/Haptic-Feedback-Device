#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

#from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
#from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


from math import cos, sin, pi, sqrt
from numpy import arccos, arctan, arctan2, transpose, array
from numpy.linalg import norm, inv
import numpy as np
import math

# ================================== Models

#Dimentions
a1 = 0.093
a2 = 0.100
a3 = 0.100
a4 = 0.093
a5 = 0.050

# Mechanics properties 
spring = 10
damper = 1

# Spheres
spheres_info = np.genfromtxt('/home/telecom/haptic_ws/src/haptic_nodes/scripts/maze1_HighCost.csv', delimiter=',')

#spheres_info = (array([[0.1,0.08,0.69],[0.1,-0.36,0.66],[0.1,-0.51,0.69],[0.1,-0.65,0.70],[0.1,0.35,0.36]]))/10

spheres_info = 10*spheres_info

class Haptic_Loop(Node):

    def __init__(self):
        # Inherance
        super().__init__('haptic_loop_class')

        # ==================================== Initial values of variables
        self.t_previous = 0
        self.theta1_previous = 0.4688
        self.theta5_previous = 2.6487

        # Initial values
        self.xe = -0.1185
        self.ye = 0.6
        self.theta1 = 0.4688
        self.theta5 = 2.6487

        # Filter parameters 
        self.torque0 = 0.0
        self.torque0_pre = 0.0
        self.torque0_filtered = 0.0


        self.torque1 = 0.0
        self.torque1_pre = 0.0
        self.torque1_filtered = 0.0

        # Send data to Controller
        #self.effort_publisher = self.create_publisher(Float64MultiArray, '/forward_effort_controller/commands', 1)

        # Publish the effort topic to evaluate
        self.effort_publisher = self.create_publisher(Float64MultiArray, '/torque_topic', 1)

        # Publishing the Arrows
        self.arrow_publisher = self.create_publisher(Marker, '/arrow_marker', 1)
        self.timer_arrow = 0.05
        self.timer_arrow = self.create_timer(self.timer_arrow, self.arrow_publish)


        
        # Ploting the two spheres into PLojuggler
        #self.sphere1_plotjuggler_publisher = self.create_publisher(Pose,'/sphere1_juggler',1)
        #self.sphere2_plotjuggler_publisher = self.create_publisher(Pose,'/sphere2_juggler',1)          

        # Obtain the Encoder Angles ===================================        
        self.encoder_angles_subscription = self.create_subscription(Float64MultiArray,'/encoder_angles', self.encoder_angles_callback, 1)
        self.encoder_angles_subscription      
        # =============================================================  
        
        # Call the Haptic Loop via callback function ====================== 
        # Timer        
        self.timer_period = 0.002  # seconds   
        self.t = self.timer_period
        self.timer = self.create_timer(self.timer_period, self.haptic_loop_function)
        # # =============================================================     

    def encoder_angles_callback(self, encoder_angles_msg):
        try:
            # Econder angles
            self.theta1 = encoder_angles_msg.data[0]   
            self.theta5 = encoder_angles_msg.data[1]

            # Pseudo Movement
            self.xe = encoder_angles_msg.data[2]
            self.ye = encoder_angles_msg.data[3]
        except:
            pass
        #self.get_logger().info(f'The Encoder angles are: {encoder_angles_msg.data}')

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

        # Convert to degrees
        #q1 = 180*(theta1/pi)
        #q2 = 180*(theta5/pi)

        return theta1, theta2, theta3, theta4, theta5

    def jacobian(self, q1, q5):
        # position of P2 and P4
        P2 = np.array([a1 * np.cos(q1), a1 * np.sin(q1)])
        P4 = np.array([a4 * np.cos(q5) - a5, a4 * np.sin(q5)])
        x2, y2 = P2
        x4, y4 = P4

        d = np.linalg.norm(P2 - P4)
        b = ((a2 * 2) - (a3 * 2) + (d ** 2)) / (2 * d)
        h = np.sqrt((a2 * 2) - (b * 2))

        d1x2 = (-a1) * np.sin(q1)
        d1y2 = a1 * np.cos(q1)
        d5x4 = (-a4) * np.sin(q5)
        d5y4 = a4 * np.cos(q5)
        dx2 = np.array([d1x2, 0])
        dy2 = np.array([d1y2, 0])
        dx4 = np.array([0, d5x4])
        dy4 = np.array([0, d5y4])

        dd = (((x4 - x2) * (dx4 - dx2)) + ((y4 - y2) * (dy4 - dy2))) / d
        db = dd - ((dd * (a2 * 2 - a3 * 2 + d * 2)) / (2 * d * 2))
        dh = -(b * db / h)
        dyh = dy2 + ((y4 - y2) / d ** 2) * (db * d - dd * b) + (dy4 - dy2) * (b / d)
        dxh = dx2 + ((x4 - x2) / d ** 2) * (db * d - dd * b) + (dx4 - dx2) * (b / d)

        dy3 = dyh - ((x4 - x2) / d ** 2) * (dh * d - dd * h) - (dx4 - dx2) * (h / d)
        dx3 = dxh + ((y4 - y2) / d ** 2) * (dh * d - dd * h) + (dy4 - dy2) * (h / d)
        J = np.vstack([dx3, dy3])

        return J

    def arrow_publish(self):

        #Jacobian Matrix
        J = array(self.jacobian(self.theta1,self.theta5)) 

        # Torque
        torque = array([self.torque0_filtered, self.torque1_filtered])
        
        # Convert
        force = (J.transpose())*torque        

        #self.get_logger().info(f'force = {float(force[0][0])}')

        # Create a marker message
        marker = Marker()
        marker.header.frame_id = "world"  # Set the frame ID
        marker.type = Marker.ARROW  # Set the marker type to arrow
        marker.action = Marker.ADD  # Set the marker action to add

        # Set the marker scale
        marker.scale.x = 0.01  # Arrow length
        marker.scale.y = 0.05  # Arrow width
        marker.scale.z = 0.05  # Arrow height

        # Set the marker color
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue

        # Set the arrow direction (from point to point)
        start_point = Point()
        start_point.x = self.xe
        start_point.y = self.ye
        start_point.z = 0.15

        end_point = Point()
        end_point.x = self.xe + float(force[0][1])
        end_point.y = self.ye + float(force[0][0]) 
        end_point.z = 0.15

        # Set the arrow points
        marker.points.append(start_point)
        marker.points.append(end_point)

        # Publish the marker
        marker.header.stamp = self.get_clock().now().to_msg()
        self.arrow_publisher.publish(marker)

    def haptic_loop_function(self):

        #========================================================  End-Eff Position and Velocity ===================================================
        #self.xe, self.ye = self.fkm(theta1,theta5)
        Pend_eff = array([self.xe,self.ye]) #Testing in Pseudo Movement

        #Jacobian Matrix
        J = array(self.jacobian(self.theta1,self.theta5)) 

        # Time interval
        dt = self.t - self.t_previous
        self.t_previous = self.t

        #Computing the Angular velocities w1 and w5
        w1 = (self.theta1 - self.theta1_previous)/(dt)
        self.theta1_previous = self.theta1

        w5 = (self.theta5 - self.theta5_previous)/(dt)
        self.theta5_previous = self.theta5

        # End-Effector velocity
        Vend_eff = J.dot(array([w1,w5]))
        #============================================================================================================================================
        #========================================================  HAPTIC LOOP ======================================================================
        for k in range(len(spheres_info)):
            center_spheres = array([spheres_info[k][1], spheres_info[k][2]])

            dist = norm(array(Pend_eff - center_spheres))

            if dist < spheres_info[k][0]:  

                vec_radius = array(Pend_eff - center_spheres)        
                modulus_vec_radius = norm(vec_radius)
                direction_vec_radius = vec_radius/modulus_vec_radius

                modulus_strain = spheres_info[k][0] - modulus_vec_radius
                strain = modulus_strain*direction_vec_radius

                force = spring*strain - damper*Vend_eff

                #self.get_logger().info(f'Force =================== {len(center_spheres)}')
                #Exit for to send the forces quickly
                break
            else:
                force = array([0.0,0.0])
        #============================================================================================================================================
        #============================================== COMPUTE THE TORQUE AND PUBLISH IT ===========================================================

        Jinv_transp = inv(J.transpose())
        #Jinv_transp = J.transpose()        
        torque = Jinv_transp.dot(force) 

        # =========================================== Filtering the Outliers ===============================================

        # =============================================> TORQUE 0
        self.torque0 = float(torque[0])
        if self.torque0 == 0.0:
            #Publish
            self.torque0_filtered = self.torque0         
        else:
            if self.torque0_pre == 0.0:
                # It means the first outlier   ===> Not Publish
                pass                
            else:                
                if (self.torque0 - self.torque0_pre) == 0: # it means that you avoided the transition, where is the outliears!
                    # Publish
                    self.torque0_filtered = self.torque0          
        self.torque0_pre = self.torque0

        # =============================================> TORQUE 1
        self.torque1 = float(torque[1])
        if self.torque1 == 0.0:
            #Publish
            self.torque1_filtered = self.torque1     
        else:
            if self.torque1_pre == 0.0:
                # It means the first outlier   ===> Not Publish
                pass                
            else:                
                if (self.torque1 - self.torque1_pre) == 0.0: # it means that you avoided the transition, where is the outliears!
                    # Publish
                    self.torque1_filtered = self.torque1          
        self.torque1_pre = self.torque1


        # =============================================> Publishing THE TORQUE
        torque_force_msg = Float64MultiArray()
        torque_force_msg.data = [self.torque0_filtered, self.torque1_filtered]
        self.effort_publisher.publish(torque_force_msg)

        # Loop Timer
        self.t += self.timer_period

        # =============================================> Creating a Force Arrow





        '''
        # Ploting the two spheres into Plotjuggler
        #============================================================   Sphere1
        x_s1 = radius*math.cos(2*self.t) + center_spheres[1][0]
        y_s1 = radius*math.sin(2*self.t) + center_spheres[1][1]
        
        sphere1_msg = Pose()
        sphere1_msg._position.x = x_s1
        sphere1_msg._position.y = y_s1
        sphere1_msg._position.z = 0.0
        self.sphere1_plotjuggler_publisher.publish(sphere1_msg)

        #==============================================================  Sphere2
        x_s2 = radius*math.cos(2*self.t) + center_spheres[2][0]
        y_s2 = radius*math.sin(2*self.t) + center_spheres[2][1]
        
        sphere2_msg = Pose()
        sphere2_msg._position.x = x_s2
        sphere2_msg._position.y = y_s2
        sphere2_msg._position.z = 0.0
        self.sphere2_plotjuggler_publisher.publish(sphere2_msg) '''

def main(args=None):
    rclpy.init(args=args)

    haptic_loop_class = Haptic_Loop()

    rclpy.spin(haptic_loop_class)

    haptic_loop_class.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()