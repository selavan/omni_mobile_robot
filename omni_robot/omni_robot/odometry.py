#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped, Vector3Stamped, Vector3
from custom_messages.msg import MotorFeedback
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.node import Node
from std_msgs.msg import Bool
import numpy as np
import time
import math
import tf2_ros
from custom_messages.srv import ResetOdom
        
class OdometryNode(Node):
    def __init__(self):
        super().__init__('Odometry_Node')
        
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.odometry_reset_subscription = self.create_subscription(Bool, 'reset_odometry', self.reset_odometry_callback, 10)
        
        self.offset_subscription = self.create_subscription(Vector3, 'offset', self.offset_callback, 10)
        
        self.encoder_subscription = self.create_subscription(
            MotorFeedback,
            '/motor_feedback',
            self.encoder_callback,
            10)
            
        self.yaw_subscription = self.create_subscription(
            Vector3Stamped,
            '/yaw_oem',
            self.yaw_callback,
            10)
        
        self.wheelposition = np.array([0.0,0.0,0.0,0.0])
        self.wheelspeed = np.array([0.0,0.0,0.0,0.0])
        self.receive = np.array([0,0,0,0])
        
        self.yaw_start = None
        self.yaw = 0.0
        self.raw_yaw = 0.0
        
        self.Vx = 0.0
        self.Vy = 0.0
        self.Wz = 0.0

        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0
        
        self.x_offset = 0.0
        self.y_offset = 0.0
        
        #global velocity
        self.Vx_global = 0.0
        self.Vy_global = 0.0
        
        self.current_time = time.perf_counter()
        self.previous_time = time.perf_counter()
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.odom_timer = self.create_timer(0.01, self.odom_timer_callback) 
        
        self.r = 0.1 / 2
        self.lx = 0.29 / 2
        self.ly = 0.265 / 2

        self.Vx_scale = 517.0 / 540.0
        self.Vy_scale = 334.0 / 355.0

        self.now = self.get_clock().now().to_msg()
        
    def offset_callback(self, msg):
        self.x_offset = msg.x
        self.y_offset = msg.y

    def reset_odometry_callback(self, msg):
        self.x = 0.0    #0.35
        self.y = 0.0
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.yaw_start = self.raw_yaw

    def yaw_callback(self, yaw_msg):
        if self.yaw_start is None:
            self.yaw_start = yaw_msg.vector.z
            
        self.yaw = yaw_msg.vector.z - self.yaw_start
        self.raw_yaw = yaw_msg.vector.z
        
    def odom_timer_callback(self):
        self.now = self.get_clock().now().to_msg()
        
        odom_msg = Odometry()
        odom_msg.header.stamp = self.now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = self.x + self.x_offset
        odom_msg.pose.pose.position.y = self.y + self.y_offset
        
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2)
    
        odom_msg.pose.covariance[0] = 1e-5
        odom_msg.pose.covariance[7] = 1e-5
        odom_msg.pose.covariance[14] = 1000000000000.0
        odom_msg.pose.covariance[21] = 1000000000000.0
        odom_msg.pose.covariance[28] = 1000000000000.0
        odom_msg.pose.covariance[35] = 1e-3
        
        odom_msg.twist.twist.linear.x = self.Vx_global
        odom_msg.twist.twist.linear.y = self.Vy_global
        
        odom_msg.twist.twist.angular.z = self.Wz
        
        odom_msg.twist.covariance[0] = 1e-5
        odom_msg.twist.covariance[7] = 1e-5
        odom_msg.twist.covariance[14] = 1000000000000.0
        odom_msg.twist.covariance[21] = 1000000000000.0
        odom_msg.twist.covariance[28] = 1000000000000.0
        odom_msg.twist.covariance[35] = 1e-3
        
        self.odom_publisher.publish(odom_msg)

        self.publish_tf()
        
        print(f"x: {self.x:.2f}, y: {self.y:.2f}, yaw: {self.yaw:.2f}")
        
    def publish_tf(self):
        joint_state = JointState()
        joint_state.header.stamp = self.now
        joint_state.name = ['w_1_joint', 'w_2_joint', 'w_3_joint', 'w_4_joint']  # Name of the joint
        joint_state.position = [self.wheelposition[0], self.wheelposition[1], self.wheelposition[2] , self.wheelposition[3]]

        # Publish the joint state
        self.joint_state_publisher.publish(joint_state)
        
        self.broadcast_transform()
        
    def callback(self):
        #forward kinematics
        self.Vx =  (self.wheelspeed[0] + self.wheelspeed[1] + self.wheelspeed[2] + self.wheelspeed[3]) * self.r / 4
        self.Vy = (-self.wheelspeed[0] + self.wheelspeed[1] - self.wheelspeed[2] + self.wheelspeed[3]) * self.r / 4
        self.Wz = (-self.wheelspeed[0] + self.wheelspeed[1] + self.wheelspeed[2] - self.wheelspeed[3]) * self.r / 4 / (self.lx + self.ly)
        
        self.Vx_global = math.cos(self.yaw) * self.Vx - math.sin(self.yaw) * self.Vy
        self.Vy_global = math.sin(self.yaw) * self.Vx + math.cos(self.yaw) * self.Vy
        
        self.current_time = time.perf_counter()
        
        Ts = self.current_time - self.previous_time
        #print(Ts)
        self.previous_time = self.current_time
        self.x+= self.Vx_global * Ts * self.Vx_scale
        self.y+= self.Vy_global * Ts * self.Vy_scale
        self.phi = self.yaw
        
    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x + self.x_offset
        t.transform.translation.y = self.y + self.y_offset
        t.transform.translation.z = 0.0  # Assuming flat ground

        if self.yaw is not None:
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(self.yaw / 2)
            t.transform.rotation.w = math.cos(self.yaw / 2)

        self.tf_broadcaster.sendTransform(t)
        
    def encoder_callback(self, msg):
        if (msg.motor_id >= 128 and msg.motor_id <=255):
            motorID = 255 - msg.motor_id
            if motorID >=1 and motorID <=4:
                self.wheelposition[motorID-1] = msg.position
                self.wheelspeed[motorID-1] = msg.speed
                self.receive[motorID-1] = 1
    	    
        if sum(self.receive) == 4:
            self.receive = np.array([0,0,0,0])
            self.callback()

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdometryNode()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    	
    	
    	
