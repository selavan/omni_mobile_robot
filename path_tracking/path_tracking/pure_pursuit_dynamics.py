import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np

# Pure Pursuit Parameters
k = 0.1  # look forward gain
Lfc = 1.0  # [m] look-ahead distance (adjusted for TurtleBot3)
Kp = 1.0  # speed proportional gain
Kd = 0.1  # derivative gain
WB = 0.16  # [m] wheelbase of TurtleBot3 Burger
target_speed = 0.2  # [m/s] target speed

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.lookahead_distance = 1.0  # Lookahead distance for Pure Pursuit
        self.current_path = None
        self.current_index = 0
        self.state = State()
        self.goal_pose = None
        self.goal_tolerance = 0.01  # [m] tolerance to consider goal reached
        self.goal_reached = False
        self.dynamic_obstacles = []

    def path_callback(self, msg):
        self.current_path = msg
        self.current_index = 0
        self.goal_pose = msg.poses[-1].pose
        self.goal_reached = False
        self.get_logger().info(f'Path received with {len(msg.poses)} waypoints.')

    def odom_callback(self, msg):
        self.state.update_from_odom(msg)

    def lidar_callback(self, msg):
        self.dynamic_obstacles = self.detect_obstacles(msg)

    def detect_obstacles(self, scan_data):
        obstacles = []
        # Implement clustering and tracking logic here
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment
        range_max = scan_data.range_max

        for i, distance in enumerate(scan_data.ranges):
            if distance < range_max:
                angle = angle_min + i * angle_increment
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                obstacles.append((x, y))

        return obstacles

    def timer_callback(self):
        if self.goal_reached:
            return

        if self.current_path is None or self.current_index >= len(self.current_path.poses):
            if self.goal_pose and self.current_index >= len(self.current_path.poses):
                self.control_to_goal()
            return

        target_pose = self.current_path.poses[self.current_index].pose

        ai = pd_control(target_speed, self.state.v, 0, 0)
        di, self.current_index = pure_pursuit_steer_control(self.state, self.current_path, self.current_index)

        self.state.update(ai, di)

        cmd_vel = Twist()
        cmd_vel.linear.x = self.state.v
        cmd_vel.angular.z = di
        self.cmd_vel_publisher.publish(cmd_vel)

        if self.state.calc_distance(target_pose.position.x, target_pose.position.y) < self.lookahead_distance:
            self.current_index += 1

        self.get_logger().info(f'Current index: {self.current_index}, Total waypoints: {len(self.current_path.poses)}, Current position: ({self.state.x}, {self.state.y})')

    def control_to_goal(self):
        if self.goal_pose is None:
            return

        distance_to_goal = self.state.calc_distance(self.goal_pose.position.x, self.goal_pose.position.y)
        if distance_to_goal < self.goal_tolerance:
            self.stop_robot()
            self.goal_reached = True
            self.get_logger().info('Goal reached. Stopping robot.')
        else:
            dx, dy = self.state.calc_derivative()
            ai = pd_control(distance_to_goal, self.state.v, 0, math.hypot(dx, dy))

            angle_to_goal = math.atan2(self.goal_pose.position.y - self.state.y, self.goal_pose.position.x - self.state.x) - self.state.yaw
            di = math.atan2(2.0 * WB * math.sin(angle_to_goal) / self.lookahead_distance, 1.0)

            cmd_vel = Twist()
            cmd_vel.linear.x = self.state.v
            cmd_vel.angular.z = di
            self.cmd_vel_publisher.publish(cmd_vel)

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)

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

def pd_control(target, current, target_derivative, current_derivative):
    return Kp * (target - current) + Kd * (target_derivative - current_derivative)

def proportional_control(target, current):
    return Kp * (target - current)

def pure_pursuit_steer_control(state, path, pind):
    cx = [pose.pose.position.x for pose in path.poses]
    cy = [pose.pose.position.y for pose in path.poses]
    ind, Lf = search_target_index(state, cx, cy, pind)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def search_target_index(state, cx, cy, pind):
    if pind is None:
        dx = [state.x - icx for icx in cx]
        dy = [state.y - icy for icy in cy]
        d = np.hypot(dx, dy)
        ind = np.argmin(d)
        pind = ind
    else:
        ind = pind
        distance_this_index = state.calc_distance(cx[ind], cy[ind])
        while True:
            if ind + 1 < len(cx):
                distance_next_index = state.calc_distance(cx[ind + 1], cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(cx) else ind
                distance_this_index = distance_next_index
            else:
                break
        pind = ind

    Lf = k * state.v + Lfc

    while Lf > state.calc_distance(cx[ind], cy[ind]):
        if (ind + 1) >= len(cx):
            break
        ind += 1

    return ind, Lf

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
