#'''
#Four Mecanum Wheels
#stop when reach the goal point, added stop function
#use PD-controller for pose, to ensure reach the goal point
#measure the actaul time from start until reach goal point
#use PI-controller for robot velocity
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import math
import numpy as np
import time

# Pure Pursuit Parameters
k = 1.0  # look forward gain
Lfc = 0.01  # [m] look-ahead distance #0.01
Kp = 0.6  # speed proportional gain
Ki = 0.001  # integral gain for PI-controller 0.1
Kd = 0.01  # derivative gain for PD-controller
target_speed = 0.4  # [m/s] target speed
lx = 0.29 / 2  # half of the robot's length in meters
ly = 0.265 / 2  # half of the robot's width in meters

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.odom_subscriber = self.create_subscription(Odometry, '/odom_estimate', self.odom_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.lookahead_distance = 0.01  # Lookahead distance for Pure Pursuit #0.01
        self.current_path = None
        self.current_index = 0
        self.state = State()
        self.goal_pose = None
        self.goal_tolerance = 0.01  # [m] tolerance to consider goal reached
        self.goal_reached = False
        self.start_time = None  # Start time of the robot movement

    def path_callback(self, msg):
        self.current_path = msg
        self.current_index = 0
        self.goal_pose = msg.poses[-1].pose
        self.goal_reached = False
        self.start_time = time.time()  # Start the timer when the path is received
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

        # Get the current target point from the path
        target_pose = self.current_path.poses[self.current_index].pose

        # Calculate the control inputs
        ai = self.pi_control(target_speed, self.state.v)
        di, self.current_index, vy = pure_pursuit_steer_control(self.state, self.current_path, self.current_index)

        # Update the state
        self.state.update(ai, di, vy)

        # Create and publish the velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = float(self.state.v)
        cmd_vel.linear.y = float(vy)
        cmd_vel.angular.z = float(di)
        self.cmd_vel_publisher.publish(cmd_vel)

        # Check if we are close enough to the target point
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
            elapsed_time = time.time() - self.start_time  # Calculate the elapsed time
            self.get_logger().info(f'Goal reached in {elapsed_time:.2f} seconds. Stopping robot.')
        else:
            # Proportional-Derivative control for linear speed based on distance to goal
            dx, dy = self.state.calc_derivative()
            ai = self.pi_control(distance_to_goal, self.state.v)

            # Proportional control for angular speed to face the goal
            angle_to_goal = math.atan2(self.goal_pose.position.y - self.state.y, self.goal_pose.position.x - self.state.x) - self.state.yaw
            di = math.atan2(2.0 * math.sin(angle_to_goal) / self.lookahead_distance, 1.0)

            # Lateral control
            vy = pd_control(self.goal_pose.position.y - self.state.y, self.state.vy, 0, 0)

            # Create and publish the velocity command
            cmd_vel = Twist()
            cmd_vel.linear.x = float(self.state.v)
            cmd_vel.linear.y = float(vy)
            cmd_vel.angular.z = float(di)
            self.cmd_vel_publisher.publish(cmd_vel)

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)

    def pi_control(self, target, current):
        error = target - current
        self.state.integral_error += error  # Update integral error
        return Kp * error + Ki * self.state.integral_error

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.vy = 0.0  # Lateral velocity
        self.integral_error = 0.0  # Integral error for PI-controller

    def update_from_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion(orientation)
        self.v = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y

    def update(self, a, delta, vy):
        self.x += self.v * math.cos(self.yaw) * 0.1
        self.y += self.v * math.sin(self.yaw) * 0.1
        self.yaw += self.v * delta * 0.1
        self.v += a * 0.1
        self.vy = vy

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

    delta = math.atan2(2.0 * math.sin(alpha) / Lf, 1.0)
    vy = 0  # Update this to implement lateral control if needed

    return delta, ind, vy

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

    Lf = k * state.v + Lfc  # update look ahead distance

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