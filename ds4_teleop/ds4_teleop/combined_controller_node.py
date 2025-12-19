import rclpy
from rclpy.node import Node

from ds4_driver_msgs.msg import Report
from geometry_msgs.msg import Twist
from collections import deque

class CombinedControllerNode(Node):
    def __init__(self):
        super().__init__('combined_controller_node')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Report,
            '/raw_report',
            self.listener_callback,
            10)

        self.window_size = 10
        self.lin_acc_x_values = deque(maxlen=self.window_size)
        self.lin_acc_z_values = deque(maxlen=self.window_size)

        # Controller neutral values
        self.neutral_left_x = 131
        self.neutral_left_y = 121
        self.neutral_right_x = 132
        self.neutral_right_y = 123

        # Speed limits
        self.max_linear_speed = 1.0  # m/s
        self.max_rotational_speed = 1.0  # rad/s

        # Initialize mode to 0 (stopped)
        self.mode = 0

    def listener_callback(self, msg):
        twist = Twist()

        # Check for mode button presses
        if msg.button_circle:
            self.mode = 1  # Analog control mode
        elif msg.button_triangle:
            self.mode = 2  # IMU control mode
        elif msg.button_square:
            self.mode = 3  # Combined control mode
        elif msg.button_cross:
            self.mode = 4  # Stop mode

        if self.mode == 1:
            Vx = self.calculate_velocity(msg.left_analog_y, self.neutral_left_y, self.max_linear_speed)
            Vy = self.calculate_velocity(msg.left_analog_x, self.neutral_left_x, self.max_linear_speed)
            Wz = self.calculate_velocity(msg.right_analog_x, self.neutral_right_x, self.max_rotational_speed)
            # Normalize Vx, Vy and Wz to limit the absolute velocity
            Vx, Vy, Wz = self.normalize_velocity(Vx, Vy, Wz)
            twist.linear.x = Vx
            twist.linear.y = Vy
            twist.angular.z = Wz
        elif self.mode == 2:
            # Push new data into the deques
            self.lin_acc_x_values.append(msg.lin_acc_x)
            self.lin_acc_z_values.append(msg.lin_acc_z)

            # Calculate the moving averages
            avg_lin_acc_x = sum(self.lin_acc_x_values) / len(self.lin_acc_x_values) if self.lin_acc_x_values else 0
            avg_lin_acc_z = sum(self.lin_acc_z_values) / len(self.lin_acc_z_values) if self.lin_acc_z_values else 0

            # Calculate velocities based on accelerometer values
            Vx = self.calculate_imu_velocity(avg_lin_acc_z, -3000, 7200, self.max_linear_speed)
            Vy = self.calculate_imu_velocity(avg_lin_acc_x, -5300, 6000, self.max_linear_speed)
            # Normalize Vx, Vy to limit the absolute velocity
            Vx, Vy = self.normalize_velocity(Vx, Vy)

            # Button press determines angular velocity
            if msg.button_l2 and not msg.button_r2:
                twist.angular.z = 0.3
            elif msg.button_r2 and not msg.button_l2:
                twist.angular.z = -0.3
            else:
                twist.angular.z = 0.0

            twist.linear.x = Vx
            twist.linear.y = Vy
        elif self.mode == 3:
            self.lin_acc_x_values.append(msg.lin_acc_x)
            Vx = self.calculate_velocity(msg.left_analog_y, self.neutral_left_y, self.max_linear_speed)
            avg_lin_acc_x = sum(self.lin_acc_x_values) / len(self.lin_acc_x_values) if self.lin_acc_x_values else 0
            Vy = self.calculate_imu_velocity(avg_lin_acc_x, -5300, 6000, self.max_linear_speed)
            # Normalize Vx, Vy to limit the absolute velocity
            Vx, Vy = self.normalize_velocity(Vx, Vy)

            Wz = self.calculate_velocity(msg.right_analog_x, self.neutral_right_x, self.max_rotational_speed)
            twist.linear.x = Vx
            twist.linear.y = Vy
            twist.angular.z = Wz
        elif self.mode == 4:
            # Robot stopped, no need to update velocities
            pass

        # Publish the Twist message
        self.publisher.publish(twist)

    def calculate_velocity(self, value, neutral, max_speed):
        return ((neutral - value) / (neutral)) * max_speed  # reverse the value, 0 becomes 1, 255 becomes -1 

    def calculate_imu_velocity(self, value, min_value, max_value, max_speed):
        # Clamp the value between min_value and max_value
        value = max(min_value, min(max_value, value))

        # Scale the value to the range [-max_speed, max_speed]
        value_range = max_value - min_value
        speed = ((value - min_value) / value_range) * 2 * max_speed - max_speed

        return speed

    def normalize_velocity(self, Vx, Vy, Wz=None):
        if Wz is None:
            magnitude = (Vx**2 + Vy**2)**0.5
            if magnitude > self.max_linear_speed:
                Vx = (Vx / magnitude) * self.max_linear_speed
                Vy = (Vy / magnitude) * self.max_linear_speed
            return Vx, Vy
        else:
            magnitude = (Vx**2 + Vy**2 + Wz**2)**0.5
            if magnitude > self.max_linear_speed:
                Vx = (Vx / magnitude) * self.max_linear_speed
                Vy = (Vy / magnitude) * self.max_linear_speed
                Wz = (Wz / magnitude) * self.max_linear_speed
            return Vx, Vy, Wz


def main(args=None):
    rclpy.init(args=args)

    node = CombinedControllerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

