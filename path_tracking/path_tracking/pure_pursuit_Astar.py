import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import math
import numpy as np

# Pure Pursuit Parameters
k = 1.0  # look forward gain
Lfc = 0.1  # [m] look-ahead distance (adjusted for TurtleBot3)
Kp = 1.0  # speed proportional gain
Kd = 0.1  # derivative gain
WB = 0.16  # [m] wheelbase of TurtleBot3 Burger
target_speed = 0.15  # [m/s] target speed

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.lookahead_distance = 0.1  # Lookahead distance for Pure Pursuit
        self.goal_threshold = 0.1  # Threshold distance to goal
        self.current_path = None
        self.current_index = 0
        self.state = State()
        self.goal_pose = None
        self.goal_reached = False
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()

    def path_callback(self, msg):
        self.current_path = msg
        self.current_index = 0
        self.goal_pose = msg.poses[-1].pose
        self.goal_reached = False
        self.get_logger().info(f'Path received with {len(msg.poses)} waypoints.')

    def odom_callback(self, msg):
        self.state.update_from_odom(msg)

    def timer_callback(self):
        if self.goal_reached:
            return

        if self.current_path is None or self.current_index >= len(self.current_path.poses):
            if self.goal_pose and self.current_index >= len(self.current_path.poses):
                self.control_to_goal()
            return

        lookahead_point = self.get_lookahead_point()
        if lookahead_point:
            self.follow_lookahead_point(lookahead_point)

    def get_lookahead_point(self):
        if self.state is None:
            return None

        for pose in self.current_path.poses:
            distance = self.state.calc_distance(pose.pose.position.x, pose.pose.position.y)
            if distance >= self.lookahead_distance:
                return pose

        return self.current_path.poses[-1]

    def follow_lookahead_point(self, goal):
        if self.is_goal_reached(goal):
            self.stop_robot()
            self.get_logger().info('Goal reached.')
            return

        control_signal = self.calculate_control(goal)
        self.apply_control(control_signal)

    def calculate_control(self, goal):
        if self.state is None:
            return Twist()

        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        current_x = self.state.x
        current_y = self.state.y

        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)
        robot_orientation = self.state.yaw
        angle_difference = self.normalize_angle(angle_to_goal - robot_orientation)

        control_signal = Twist()
        distance_to_goal = self.state.calc_distance(goal_x, goal_y)
        current_time = self.get_clock().now()
        time_diff = (current_time - self.previous_time).nanoseconds / 1e9
        error = distance_to_goal
        derivative = (error - self.previous_error) / time_diff if time_diff > 0 else 0.0
        stopping_signal = Kp * error + Kd * derivative

        control_signal.linear.x = min(stopping_signal, target_speed)
        control_signal.angular.z = angle_difference

        self.previous_error = error
        self.previous_time = current_time

        self.get_logger().info(f'Control signal: linear.x={control_signal.linear.x}, angular.z={control_signal.angular.z}')
        return control_signal

    def apply_control(self, control_signal):
        self.cmd_vel_publisher.publish(control_signal)

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)
        self.goal_reached = True

    def is_goal_reached(self, goal):
        if self.state is None:
            return False
        distance_to_goal = self.state.calc_distance(goal.pose.position.x, goal.pose.position.y)
        return distance_to_goal < self.goal_threshold

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update_from_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion(orientation)
        self.v = msg.twist.twist.linear.x

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * 0.1
        self.y += self.v * math.sin(self.yaw) * 0.1
        self.yaw += self.v / WB * math.tan(delta) * 0.1
        self.v += a * 0.1

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)

    def calc_derivative(self):
        return self.v * math.cos(self.yaw), self.v * math.sin(self.yaw)

def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuitNode()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import math
import atexit

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.path_sub = self.create_subscription(Path, 'planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.executed_path_pub = self.create_publisher(Path, 'executed_path', 10)
        self.path = []
        self.current_pose = None
        self.lookahead_distance = 0.1  # Lookahead distance for the pure pursuit algorithm
        self.goal_threshold = 0.1  # Threshold distance to goal

        # PD control parameters
        self.kp = 0.7  # Proportional gain
        self.kd = 0.6  # Derivative gain
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()

        # Timer to periodically check and follow the path
        self.timer = self.create_timer(0.1, self.follow_path)

        # Register the shutdown hook with atexit
        atexit.register(self.stop_robot)

    def path_callback(self, msg):
        self.path = msg.poses
        self.get_logger().info('Received smoothed path with %d points' % len(self.path))

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def follow_path(self):
        if not self.path or self.current_pose is None:
            return

        lookahead_point = self.get_lookahead_point()
        if lookahead_point:
            self.follow_lookahead_point(lookahead_point)

    def get_lookahead_point(self):
        if self.current_pose is None:
            return None

        for pose in self.path:
            distance = self.calculate_distance(self.current_pose, pose.pose)
            if distance >= self.lookahead_distance:
                return pose

        # If no lookahead point is found, return the final goal
        return self.path[-1]

    def follow_lookahead_point(self, goal):
        if self.is_goal_reached(goal):
            self.stop_robot()
            self.get_logger().info('Goal reached.')
            return

        control_signal = self.calculate_control(goal)
        self.apply_control(control_signal)

    def calculate_control(self, goal):
        if self.current_pose is None:
            return Twist()

        goal_pose = goal.pose
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)
        robot_orientation = self.get_yaw_from_quaternion(self.current_pose.orientation)
        angle_difference = self.normalize_angle(angle_to_goal - robot_orientation)

        control_signal = Twist()
        distance_to_goal = self.calculate_distance(self.current_pose, goal_pose)
        current_time = self.get_clock().now()
        time_diff = (current_time - self.previous_time).nanoseconds / 1e9
        error = distance_to_goal
        derivative = (error - self.previous_error) / time_diff if time_diff > 0 else 0.0
        stopping_signal = self.kp * error + self.kd * derivative

        control_signal.linear.x = min(stopping_signal, 0.3)
        control_signal.angular.z = angle_difference

        self.previous_error = error
        self.previous_time = current_time

        self.get_logger().info(f'Control signal: linear.x={control_signal.linear.x}, angular.z={control_signal.angular.z}')
        return control_signal

    def apply_control(self, control_signal):
        self.cmd_pub.publish(control_signal)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info('Robot stopped.')

    def calculate_distance(self, pose1, pose2):
        return math.sqrt((pose1.position.x - pose2.position.x) ** 2 + (pose1.position.y - pose2.position.y) ** 2)

    def get_yaw_from_quaternion(self, quaternion):
        # Convert quaternion to Euler angles
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def is_goal_reached(self, goal):
        if self.current_pose is None:
            return False
        distance_to_goal = self.calculate_distance(self.current_pose, goal.pose)
        return distance_to_goal < self.goal_threshold

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.stop_robot()  # Ensure the robot stops when the node is destroyed
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''



'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import math
import atexit

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.path_sub = self.create_subscription(Path, 'planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.executed_path_pub = self.create_publisher(Path, 'executed_path', 10)
        self.path = []
        self.current_pose = None
        self.lookahead_distance = 0.1  # Lookahead distance for the pure pursuit algorithm
        self.goal_threshold = 0.1  # Threshold distance to goal

        # PD control parameters
        self.kp = 0.7  # Proportional gain
        self.kd = 0.6  # Derivative gain
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()

        # Timer to periodically check and follow the path
        self.timer = self.create_timer(0.1, self.follow_path)

        # Register the shutdown hook with atexit
        atexit.register(self.stop_robot)

    def path_callback(self, msg):
        self.path = msg.poses
        self.get_logger().info('Received smoothed path with %d points' % len(self.path))

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def follow_path(self):
        if not self.path or self.current_pose is None:
            return

        lookahead_point = self.get_lookahead_point()
        if lookahead_point:
            self.follow_lookahead_point(lookahead_point)

    def get_lookahead_point(self):
        if self.current_pose is None:
            return None

        for pose in self.path:
            distance = self.calculate_distance(self.current_pose, pose.pose)
            if distance >= self.lookahead_distance:
                return pose

        # If no lookahead point is found, return the final goal
        return self.path[-1]

    def follow_lookahead_point(self, goal):
        control_signal = self.calculate_control(goal)
        self.apply_control(control_signal)

        # Check if the final goal is reached after applying control
        if self.is_goal_reached(goal):
            self.stop_robot()
            self.get_logger().info('Goal reached.')

    def calculate_control(self, goal):
        if self.current_pose is None:
            return Twist()

        goal_pose = goal.pose
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)
        robot_orientation = self.get_yaw_from_quaternion(self.current_pose.orientation)
        angle_difference = self.normalize_angle(angle_to_goal - robot_orientation)

        control_signal = Twist()
        distance_to_goal = self.calculate_distance(self.current_pose, goal_pose)
        current_time = self.get_clock().now()
        time_diff = (current_time - self.previous_time).nanoseconds / 1e9
        error = distance_to_goal
        derivative = (error - self.previous_error) / time_diff if time_diff > 0 else 0.0
        stopping_signal = self.kp * error + self.kd * derivative

        if distance_to_goal < self.goal_threshold:
            control_signal.linear.x = 0.0
            control_signal.angular.z = 0.0
        else:
            control_signal.linear.x = min(stopping_signal, 0.3)
            control_signal.angular.z = angle_difference

        self.previous_error = error
        self.previous_time = current_time

        self.get_logger().info(f'Control signal: linear.x={control_signal.linear.x}, angular.z={control_signal.angular.z}')
        return control_signal

    def apply_control(self, control_signal):
        self.cmd_pub.publish(control_signal)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info('Robot stopped.')

    def calculate_distance(self, pose1, pose2):
        return math.sqrt((pose1.position.x - pose2.position.x) ** 2 + (pose1.position.y - pose2.position.y) ** 2)

    def get_yaw_from_quaternion(self, quaternion):
        # Convert quaternion to Euler angles
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def is_goal_reached(self, goal):
        if self.current_pose is None:
            return False
        distance_to_goal = self.calculate_distance(self.current_pose, goal.pose)
        return distance_to_goal < self.goal_threshold

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.stop_robot()  # Ensure the robot stops when the node is destroyed
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
