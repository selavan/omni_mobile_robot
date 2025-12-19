import rclpy
from rclpy.node import Node

from ds4_driver_msgs.msg import Report
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import IMU

import math
import time

class IMUControllerNode(Node):
    def __init__(self):
        super().__init__('imu_controller_node')

        self.publisher = self.create_publisher(Twist, '/body_velocity', 10)
        
        self.subscription_report = self.create_subscription(
            Report,
            '/raw_report',
            self.report_listener_callback,
            10)
            
        self.subscription_imu_yaw = self.create_subscription(
            Float32,
            '/HFI_A9_yaw',
            self.imu_yaw_listener_callback,
            10)

        # Initialize current_orientation and desired_orientation
        self.current_orientation = 0.0
        self.desired_orientation = 0.0
        
        self.LJX = 0.0
        self.LJY = 0.0
        self.RJX = 0.0
        self.RJY = 0.0
        
        self.max_Vx = 3.0
        self.max_Vy = 3.0
        self.max_Wz = 3.0

        self.p = 3.0  # Proportional control gain
        
        self.stop = 0
        
        self.Vx = 0.0
        self.Vy = 0.0
        self.Wz = 0.0
        
        self.full_rotations = 0
        self.previous_raw_yaw_mag = None

    def imu_yaw_listener_callback(self, msg):
        self.current_orientation = msg.data
        
        raw_yaw_mag = math.atan2(-corrected_mag_y, corrected_mag_x)

        # Initialize previous_raw_yaw_mag on the first run
        if self.previous_raw_yaw_mag is None:
            self.previous_raw_yaw_mag = raw_yaw_mag

        yaw_diff = raw_yaw_mag - self.previous_raw_yaw_mag

        # Check for full rotation
        if yaw_diff > math.pi:
            self.full_rotations -= 1
        elif yaw_diff < -math.pi:
            self.full_rotations += 1

        # Update continuous yaw
        self.yaw_mag = raw_yaw_mag + (self.full_rotations * 2 * math.pi)
        self.previous_raw_yaw_mag = raw_yaw_mag

    def report_listener_callback(self, msg):
        twist = Twist()
        
        LJX = msg.left_analog_y;
        if LJX > 110 and LJX < 148:
            self.Vx = 0.0
        else:
           self.Vx = -self.max_Vx*(LJX - 128) / 128
        
        LJY = msg.left_analog_x;
        if LJY > 110 and LJY < 148:
            self.Vy = 0.0
        else:
           self.Vy = -self.max_Vy*(LJY - 128) / 128
            
        # Button press determines desired orientation
        
        self.Wz = 0.0
        if msg.button_l2 and not msg.button_r2:
            #self.desired_orientation = 0.5
            self.Wz = 0.5
        elif msg.button_r2 and not msg.button_l2:
            self.Wz = -0.5
            
        if msg.button_cross == 1:
            self.stop += 1
            time.sleep(0.3)
            
        if self.stop % 2 == 1:
            self.Vx = 0.0
            self.Vy = 0.0
            self.Wz = 0.0
            
        print(self.Vx, self.Vy, self.Wz)        
        #self.Vx_f = self.Vx_f - 0.1 * (self.Vx_f - Vx)
        #self.Vy_f = self.Vy_f - 0.1 * (self.Vy_f - Vy)
        #self.Wz_f = self.Wz_f - 0.1 * (self.Wz_f - Wz)

        twist.linear.x = self.Vx
        twist.linear.y = self.Vy
        twist.angular.z = self.Wz

        # Publish the Twist message
        self.publisher.publish(twist)
        

def main(args=None):
    rclpy.init(args=args)

    imu_controller = IMUControllerNode()

    rclpy.spin(imu_controller)

    imu_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

