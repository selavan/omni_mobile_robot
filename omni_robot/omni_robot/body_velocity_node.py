#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_messages.msg import MotorCommand
from geometry_msgs.msg import Twist
import math

class BodyVelocityNode(Node):

    def __init__(self):
        super().__init__('body_velocity_node')
        #self.width = 0.29 / 2
        #self.height = 0.26 / 2
        
        #self.r = 0.1 / 2
        
        #self.a = math.sqrt(2) / (2 * self.r)
        #self.b = self.l / self.r
        
        self.r = 0.1 / 2    #0.1m
        self.lx = 0.29 / 2
        self.ly = 0.265 / 2

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(MotorCommand, '/publish_motor', 10)
        
    def inverse_kinematics(self, Vx, Vy, omega_z):
        wheel_speed = [
            (Vx - Vy - (self.lx + self.ly) * omega_z) / self.r, 
            (Vx + Vy + (self.lx + self.ly) * omega_z) / self.r, 
            (Vx - Vy + (self.lx + self.ly) * omega_z) / self.r, 
            (Vx + Vy - (self.lx + self.ly) * omega_z) / self.r, 
        ]
        return wheel_speed

    def listener_callback(self, msg):
        Vx = msg.linear.x
        Vy = msg.linear.y
        omega_z = msg.angular.z

        wheel_speeds = self.inverse_kinematics(Vx, Vy, omega_z)
        
        for i in range(4):
            MotorMsg = MotorCommand()
            MotorMsg.speedmode = True
            MotorMsg.motor_id = i+1
            MotorMsg.goal = wheel_speeds[i]
            self.publisher_.publish(MotorMsg)

def main(args=None):
    rclpy.init(args=args)
    body_velocity_node = BodyVelocityNode()
    rclpy.spin(body_velocity_node)
    body_velocity_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

