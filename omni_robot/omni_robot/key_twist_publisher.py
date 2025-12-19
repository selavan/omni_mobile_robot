import pygame
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyTwistPublisher(Node):
    def __init__(self):
        super().__init__('key_twist_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(0.01, self.publish_twist)
        self.get_logger().info('Twist publisher node has been initialized.')
        self.key_to_twist_mapping = {
            pygame.K_UP: (1.0, 0.0),
            pygame.K_DOWN: (-1.0, 0.0),
            pygame.K_LEFT: (0.0, 1.0),
            pygame.K_RIGHT: (0.0, -1.0)
        }
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def publish_twist(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_vel
        twist_msg.angular.z = self.angular_vel
        self.publisher_.publish(twist_msg)

    def handle_keyboard_event(self, event):
        if event.type == pygame.KEYDOWN:
            if event.key in self.key_to_twist_mapping:
                self.linear_vel, self.angular_vel = self.key_to_twist_mapping[event.key]
        elif event.type == pygame.KEYUP:
            if event.key in self.key_to_twist_mapping:
                self.linear_vel, self.angular_vel = 0.0, 0.0


def main():
    pygame.init()
    display = pygame.display.set_mode((300, 300))
    rclpy.init()
    key_twist_publisher = KeyTwistPublisher()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
                return

            key_twist_publisher.handle_keyboard_event(event)

        key_twist_publisher.publish_twist()
        rclpy.spin_once(key_twist_publisher)


if __name__ == '__main__':
    main()

'''
import serial, time, pygame, sys, threading
import numpy as np

pygame.init()
display = pygame.display.set_mode((300, 300))

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
            print(event.type)

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                print("q")
            elif event.key == pygame.K_a:
                print("a")
            elif event.key == pygame.K_d:
                print("d")
            elif event.key == pygame.K_s:
                print("s")
            elif event.key == pygame.K_UP:
                print("up")
            elif event.key == pygame.K_DOWN:
                print("down")
            elif event.key == pygame.K_0:
                print("0")
            break
import pygame
import rclpy
from std_msgs.msg import String
'''
'''
def pygame_keyboard_publisher():
    pygame.init()
    display = pygame.display.set_mode((300, 300))
    rclpy.init()
    node = rclpy.create_node('pygame_keyboard_publisher')
    publisher = node.create_publisher(String, 'motor_control', 10)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
                return

            if event.type == pygame.KEYDOWN:
                key_code = event.key
                message = String()
                message.data = str(key_code)
                # Publish the key code as an ROS 2 message
                publisher.publish(message)

        rclpy.spin_once(node)

if __name__ == '__main__':
    pygame_keyboard_publisher()
'''    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
                       			

