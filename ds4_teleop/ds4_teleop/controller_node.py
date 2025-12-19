import rclpy
from rclpy.node import Node
from ds4_driver_msgs.msg import Report
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import math
import time
import numpy as np

class IMUControllerNode(Node):
    def __init__(self):
        super().__init__('imu_controller_node')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_report = self.create_subscription(
            Report,
            '/raw_report',
            self.report_listener_callback,
            10)
        
        self.subscription_imu = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10)

        # Initialize current_yaw and desired_yaw
        self.current_yaw = 0.0
        self.desired_yaw = 0.0
        
        self.LJX = 0.0
        self.LJY = 0.0
        self.RJX = 0.0
        self.RJY = 0.0
        
        self.max_Vx = 0.6  #4.0
        self.max_Vy = 0.6  #4.0
        self.max_Wz = 3.0

        self.p = 6.0  # Proportional control gain
        
        self.Vx = 0.0
        self.Vy = 0.0
        self.Wz = 0.0

        self.joystick_relaxed = True
        self.full_rotations = 0.0
        self.previous_raw_yaw = 0.0
        
        self.home_yaw_set = False
        self.home_yaw = 0.0
        
        self.Vx_filter = 0.0;
        self.Vy_filter = 0.0;
        self.Wz_filter = 0.0;
        self.a = 0.1;

    def imu_callback(self, msg):
        # Assuming the orientation is provided as a quaternion
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        # Assuming euler[2] gives the yaw angle
        raw_yaw = self.quaternion_to_yaw(*quaternion)
        yaw_diff = raw_yaw - self.previous_raw_yaw
        

        if yaw_diff > math.pi:
            self.full_rotations -= 1
        elif yaw_diff < -math.pi:
            self.full_rotations += 1

        self.current_yaw = raw_yaw + (self.full_rotations * 2 * math.pi) - self.home_yaw
        
        if self.home_yaw_set == False:
            self.home_yaw_set = True
            self.home_yaw = self.current_yaw 
        self.previous_raw_yaw = raw_yaw

        print(self.current_yaw)

    def quaternion_to_yaw(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
        
        return yaw_z  # in radians

    def report_listener_callback(self, msg):
        twist = Twist()
        
        LJX = msg.left_analog_y;
        if LJX > 108 and LJX < 148:
            self.Vx = 0.0
        else:
           self.Vx = -self.max_Vx*(LJX - 128) / 128
        
        LJY = msg.left_analog_x;
        if LJY > 108 and LJY < 148:
            self.Vy = 0.0
        else:
           self.Vy = -self.max_Vy*(LJY - 128) / 128

        RJX = msg.right_analog_x

        # Map RJX (0 to 255) to -Wzmax to Wzmax
        if RJX > 108 and RJX < 148:  # Assuming 128 +/- 20 as the deadzone
            if self.joystick_relaxed:
                # Joystick is still in relaxed state
                # Implement P-controller to maintain desired_yaw
                orientation_error = self.desired_yaw - self.current_yaw
                if abs(orientation_error) < 0.05:
                    orientation_error = 0.0
                self.Wz = self.p * orientation_error
            else:
                # Joystick just moved to relaxed state
                self.desired_yaw = self.current_yaw
                self.joystick_relaxed = True
                self.Wz = 0.0
        else:
            # Joystick is active
            self.Wz = -self.max_Wz * (RJX - 128) / 128
            self.joystick_relaxed = False

        if self.Wz > self.max_Wz:
            self.Wz = self.max_Wz
        elif self.Wz < -self.max_Wz:
            self.Wz = - self.max_Wz

        #if abs(self.Wz) < 0.02:
        #    self.Wz = 0.0    

        #theta = math.pi/4
        #self.Vx_new = self.Vx * math.cos(theta) - self.Vy * math.sin(theta)
        #self.Vy_new = self.Vx * math.sin(theta) + self.Vy * math.sin(theta)
        
        
        #x_filter[i] = (x_real - (1-a) * x_filter[i-1]) / a
        
        self.Vx_filter = self.a * self.Vx + (1 - self.a) * self.Vx_filter
        self.Vy_filter = self.a * self.Vy + (1 - self.a) * self.Vy_filter
        self.Wz_filter = self.a * self.Wz + (1 - self.a) * self.Wz_filter

        
        twist.linear.x = self.Vx
        twist.linear.y = self.Vy
        twist.angular.z = self.Wz
        
        twist.linear.x = self.Vx_filter
        twist.linear.y = self.Vy_filter
        twist.angular.z = self.Wz_filter
        

        # Publish the Twist message
        self.publisher.publish(twist)
        
        #print("lin_acc_x: ", msg.lin_acc_x)
        #print("lin_acc_y: ", msg.lin_acc_y)
        #print("lin_acc_z: ", msg.lin_acc_z)
        

def main(args=None):
    rclpy.init(args=args)

    imu_controller = IMUControllerNode()

    rclpy.spin(imu_controller)

    imu_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
