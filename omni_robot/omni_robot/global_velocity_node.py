import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math
import tf2_ros
import tf2_py
import numpy as np

class VelocityControllerNode(Node):
    def __init__(self):
        super().__init__('global_velocity_node')

        # Publishers
        self.publisher = self.create_publisher(Twist, '/body_velocity', 10)

        # Subscriptions
        self.subscription_imu_yaw = self.create_subscription(
            Float32,
            '/imu_yaw',
            self.imu_yaw_callback,
            10)

        self.subscription_home_yaw = self.create_subscription(
            Float32,
            '/home_yaw',
            self.home_yaw_callback,
            10)

        self.subscription_global_velocity = self.create_subscription(
            Twist,
            '/global_velocity',
            self.global_velocity_callback,
            10)

        # Current yaw values
        self.imu_yaw = 0.0
        self.home_yaw = 0.0

        # Global velocity
        self.global_velocity = Twist()

    def imu_yaw_callback(self, msg):
        # Convert degrees to radians and save the current IMU yaw
        self.imu_yaw = -math.radians(msg.data)

    def home_yaw_callback(self, msg):
        # Save the current home yaw
        self.home_yaw = msg.data

    def global_velocity_callback(self, msg):
        # Save the current global velocity
        self.global_velocity = msg

        # Compute the body frame velocity
        body_velocity = self.convert_velocity_to_body_frame()

        # Publish the body frame velocity
        self.publisher.publish(body_velocity)

    def convert_velocity_to_body_frame(self):
        # Compute the yaw difference
        yaw_diff = self.imu_yaw - self.home_yaw

        # Create a rotation matrix
        rotation_matrix = np.array([
            [np.cos(yaw_diff), -np.sin(yaw_diff), 0],
            [np.sin(yaw_diff), np.cos(yaw_diff), 0],
            [0, 0, 1]
        ])

        # Convert the global velocity to a numpy array
        global_velocity_array = np.array([
            self.global_velocity.linear.x,
            self.global_velocity.linear.y,
            self.global_velocity.angular.z
        ])

        # Multiply the rotation matrix by the global velocity to compute the body velocity
        body_velocity_array = np.dot(rotation_matrix, global_velocity_array)

        # Convert the body velocity array to a Twist message
        body_velocity = Twist()
        body_velocity.linear.x = body_velocity_array[0]
        body_velocity.linear.y = body_velocity_array[1]
        body_velocity.angular.z = body_velocity_array[2]

        return body_velocity

def main(args=None):
    rclpy.init(args=args)

    velocity_controller = VelocityControllerNode()

    rclpy.spin(velocity_controller)

    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

