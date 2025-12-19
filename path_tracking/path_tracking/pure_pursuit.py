#'''
#Four Mecanum Wheels
#with /odom
#stop when reach the goal point, added stop function
#use PD-controller for pose to ensure reach the goal point
#v1
#measure the actaul time from start until reach goal point
#use PI-controller robot velocity
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



#'''


'''
#Four Mecanum Wheels
#with /odom
#stop when reach the goal point, added stop function
#use PD-controller for pose and vel control to ensure reach the goal point
#v1
#measure the actaul time from start until reach goal point
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import math
import numpy as np
import time

# Pure Pursuit Parameters
k = 1.0  # look forward gain
Lfc = 0.01  # [m] look-ahead distance (adjusted for TurtleBot3)
Kp = 1.0  # speed proportional gain
Kd = 0.01  # derivative gain
target_speed = 0.4  # [m/s] target speed
lx = 0.29 / 2  # half of the robot's length in meters
ly = 0.265 / 2  # half of the robot's width in meters

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        #self.odom_subscriber = self.create_subscription(Odometry, '/odom_estimate', self.odom_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.lookahead_distance = 0.01  # Lookahead distance for Pure Pursuit
        self.current_path = None
        self.current_index = 0
        self.state = State()
        self.goal_pose = None
        self.goal_tolerance = 0.03  # [m] tolerance to consider goal reached
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
        ai = pd_control(target_speed, self.state.v, 0, 0)
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
            ai = pd_control(distance_to_goal, self.state.v, 0, math.hypot(dx, dy))

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

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.vy = 0.0  # Lateral velocity

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


'''


'''
#Four Mecanum Wheels
#with /odom
#stop when reach the goal point, added stop function
#use PD-controller for pose and vel control to ensure reach the goal point
#v1

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import math
import numpy as np

# Pure Pursuit Parameters
k = 1.0  # look forward gain
Lfc = 0.05  # [m] look-ahead distance (adjusted for TurtleBot3)
Kp = 1.0  # speed proportional gain
Kd = 0.01  # derivative gain
target_speed = 0.4  # [m/s] target speed
lx = 0.29 / 2  # half of the robot's length in meters
ly = 0.265 / 2  # half of the robot's width in meters

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        #self.odom_subscriber = self.create_subscription(Odometry, '/odom_estimate', self.odom_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.lookahead_distance = 0.05  # Lookahead distance for Pure Pursuit
        self.current_path = None
        self.current_index = 0
        self.state = State()
        self.goal_pose = None
        self.goal_tolerance = 0.01  # [m] tolerance to consider goal reached
        self.goal_reached = False

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

        # Get the current target point from the path
        target_pose = self.current_path.poses[self.current_index].pose

        # Calculate the control inputs
        ai = pd_control(target_speed, self.state.v, 0, 0)
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
            self.get_logger().info('Goal reached. Stopping robot.')
        else:
            # Proportional-Derivative control for linear speed based on distance to goal
            dx, dy = self.state.calc_derivative()
            ai = pd_control(distance_to_goal, self.state.v, 0, math.hypot(dx, dy))

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

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.vy = 0.0  # Lateral velocity

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
'''



'''
#Four Mecanum Wheels
#with /odom
#stop when reach the goal point, added stop function
#use PD-controller for pose and vel control to ensure reach the goal point

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import math
import numpy as np

# Pure Pursuit Parameters
k = 1.0  # look forward gain
Lfc = 0.5  # [m] look-ahead distance (adjusted for TurtleBot3)
Kp = 1.0  # speed proportional gain
Kd = 0.01  # derivative gain
target_speed = 0.4  # [m/s] target speed
lx = 0.29/2  # half of the robot's length in meters
ly = 0.265/2  # half of the robot's width in meters

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.lookahead_distance = 0.5  # Lookahead distance for Pure Pursuit
        self.current_path = None
        self.current_index = 0
        self.state = State()
        self.goal_pose = None
        self.goal_tolerance = 0.1  # [m] tolerance to consider goal reached
        self.goal_reached = False

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

        # Get the current target point from the path
        target_pose = self.current_path.poses[self.current_index].pose

        # Calculate the control inputs
        ai = pd_control(target_speed, self.state.v, 0, 0)
        di, self.current_index, vy = pure_pursuit_steer_control(self.state, self.current_path, self.current_index)

        # Update the state
        self.state.update(ai, di, vy)

        # Create and publish the velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = self.state.v
        cmd_vel.linear.y = vy
        cmd_vel.angular.z = di
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
            self.get_logger().info('Goal reached. Stopping robot.')
        else:
            # Proportional-Derivative control for linear speed based on distance to goal
            dx, dy = self.state.calc_derivative()
            ai = pd_control(distance_to_goal, self.state.v, 0, math.hypot(dx, dy))

            # Proportional control for angular speed to face the goal
            angle_to_goal = math.atan2(self.goal_pose.position.y - self.state.y, self.goal_pose.position.x - self.state.x) - self.state.yaw
            di = math.atan2(2.0 * math.sin(angle_to_goal) / self.lookahead_distance, 1.0)

            # Lateral control
            vy = pd_control(self.goal_pose.position.y - self.state.y, self.state.vy, 0, 0)

            # Create and publish the velocity command
            cmd_vel = Twist()
            cmd_vel.linear.x = self.state.v
            cmd_vel.linear.y = vy
            cmd_vel.angular.z = di
            self.cmd_vel_publisher.publish(cmd_vel)

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.vy = 0.0  # Lateral velocity

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
    else:  # toward goal
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    delta = math.atan2(2.0 * math.sin(alpha) / Lf, 1.0)
    vy = 0  # Update this to implement lateral control if needed

    return delta, ind, vy

def search_target_index(state, cx, cy, pind):
    if pind is None:
        # search nearest point index
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

    # search look ahead target point index
    while Lf > state.calc_distance(cx[ind], cy[ind]):
        if (ind + 1) >= len(cx):
            break  # not exceed goal
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


'''

'''
#Four Mecanum Wheels
#with /odom
#stop when reach the goal point, added stop function
#use P-controller to ensure reach the goal point
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import math
import numpy as np

# Pure Pursuit Parameters
k = 1.0  # look forward gain
Lfc = 1.0  # [m] look-ahead distance
Kp = 5.0  # speed proportional gain
target_speed = 0.3  # [m/s] target speed

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        #self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom_estimate', self.odom_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.lookahead_distance = 1.0  # Lookahead distance for Pure Pursuit
        self.current_path = None
        self.current_index = 0
        self.state = State()
        self.goal_reached = False

    def path_callback(self, msg):
        self.current_path = msg
        self.current_index = 0
        self.goal_reached = False
        self.get_logger().info(f'Path received with {len(msg.poses)} waypoints.')

    def odom_callback(self, msg):
        self.state.update_from_odom(msg)

    def timer_callback(self):
        if self.current_path is None or self.goal_reached:
            return

        if self.current_index >= len(self.current_path.poses):
            self.stop_robot()
            self.get_logger().info('Goal reached. Stopping robot.')
            self.goal_reached = True
            return

        # Get the current target point from the path
        target_pose = self.current_path.poses[self.current_index].pose

        # Calculate the control inputs
        ai = proportional_control(target_speed, self.state.v)
        di, self.current_index, vy = pure_pursuit_steer_control(self.state, self.current_path, self.current_index)

        # Update the state
        self.state.update(ai, di, vy)

        # Create and publish the velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = self.state.v
        cmd_vel.linear.y = float(vy)  # Ensure vy is a float
        cmd_vel.angular.z = di
        self.cmd_vel_publisher.publish(cmd_vel)

        # Check if we are close enough to the target point
        if self.state.calc_distance(target_pose.position.x, target_pose.position.y) < self.lookahead_distance:
            self.current_index += 1

        self.get_logger().info(f'Current index: {self.current_index}, Total waypoints: {len(self.current_path.poses)}, Current position: ({self.state.x}, {self.state.y})')

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.vy = 0.0  # Lateral velocity

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
    else:  # toward goal
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    delta = math.atan2(2.0 * math.sin(alpha) / Lf, 1.0)
    vy = 0  # Update this to implement lateral control if needed

    return delta, ind, vy

def search_target_index(state, cx, cy, pind):
    if pind is None:
        # search nearest point index
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

    # search look ahead target point index
    while Lf > state.calc_distance(cx[ind], cy[ind]):
        if (ind + 1) >= len(cx):
            break  # not exceed goal
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

'''

'''
#Turtlebot3
#with /odom
#stop when reach the goal point, added stop function
#use PD-controller to ensure reach the goal point
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
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.lookahead_distance = 0.1  # Lookahead distance for Pure Pursuit
        self.current_path = None
        self.current_index = 0
        self.state = State()
        self.goal_pose = None
        self.goal_tolerance = 0.01  # [m] tolerance to consider goal reached
        self.goal_reached = False

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

        # Get the current target point from the path
        target_pose = self.current_path.poses[self.current_index].pose

        # Calculate the control inputs
        ai = pd_control(target_speed, self.state.v, 0, 0)
        di, self.current_index = pure_pursuit_steer_control(self.state, self.current_path, self.current_index)

        # Update the state
        self.state.update(ai, di)

        # Create and publish the velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = self.state.v
        cmd_vel.angular.z = di
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
            self.get_logger().info('Goal reached. Stopping robot.')
        else:
            # Proportional-Derivative control for linear speed based on distance to goal
            dx, dy = self.state.calc_derivative()
            ai = pd_control(distance_to_goal, self.state.v, 0, math.hypot(dx, dy))

            # Proportional control for angular speed to face the goal
            angle_to_goal = math.atan2(self.goal_pose.position.y - self.state.y, self.goal_pose.position.x - self.state.x) - self.state.yaw
            di = math.atan2(2.0 * WB * math.sin(angle_to_goal) / self.lookahead_distance, 1.0)

            # Create and publish the velocity command
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
    else:  # toward goal
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def search_target_index(state, cx, cy, pind):
    if pind is None:
        # search nearest point index
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

    # search look ahead target point index
    while Lf > state.calc_distance(cx[ind], cy[ind]):
        if (ind + 1) >= len(cx):
            break  # not exceed goal
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

'''

'''
#Turtlebot3
#with /odom
#stop when reach the goal point, added stop function
#use P-controller to ensure reach the goal point
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import math
import numpy as np

# Pure Pursuit Parameters
k = 1.0  # look forward gain
Lfc = 0.05  # [m] look-ahead distance (adjusted for TurtleBot3)
Kp = 1.0  # speed proportional gain
WB = 0.16  # [m] wheelbase of TurtleBot3 Burger
target_speed = 0.22  # [m/s] target speed

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.lookahead_distance = 0.05  # Lookahead distance for Pure Pursuit
        self.current_path = None
        self.current_index = 0
        self.state = State()
        self.goal_pose = None
        self.goal_tolerance = 0.02  # [m] tolerance to consider goal reached

    def path_callback(self, msg):
        self.current_path = msg
        self.current_index = 0
        self.goal_pose = msg.poses[-1].pose
        self.get_logger().info(f'Path received with {len(msg.poses)} waypoints.')

    def odom_callback(self, msg):
        self.state.update_from_odom(msg)

    def timer_callback(self):
        if self.current_path is None or self.current_index >= len(self.current_path.poses):
            if self.goal_pose and self.current_index >= len(self.current_path.poses):
                self.control_to_goal()
            return

        # Get the current target point from the path
        target_pose = self.current_path.poses[self.current_index].pose

        # Calculate the control inputs
        ai = proportional_control(target_speed, self.state.v)
        di, self.current_index = pure_pursuit_steer_control(self.state, self.current_path, self.current_index)

        # Update the state
        self.state.update(ai, di)

        # Create and publish the velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = self.state.v
        cmd_vel.angular.z = di
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
            self.get_logger().info('Goal reached. Stopping robot.')
        else:
            # Proportional control for linear speed based on distance to goal
            ai = proportional_control(distance_to_goal, self.state.v)

            # Proportional control for angular speed to face the goal
            angle_to_goal = math.atan2(self.goal_pose.position.y - self.state.y, self.goal_pose.position.x - self.state.x) - self.state.yaw
            di = math.atan2(2.0 * WB * math.sin(angle_to_goal) / self.lookahead_distance, 1.0)

            # Create and publish the velocity command
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
    else:  # toward goal
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def search_target_index(state, cx, cy, pind):
    if pind is None:
        # search nearest point index
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

    # search look ahead target point index
    while Lf > state.calc_distance(cx[ind], cy[ind]):
        if (ind + 1) >= len(cx):
            break  # not exceed goal
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

'''

'''
#with /odom
#stop when reach the goal point, added stop function
#for Turtlebot3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import math
import numpy as np

# Pure Pursuit Parameters
Lfc = 1.5  # [m] look-ahead distance (adjusted for TurtleBot3)
k = 1.0  # look forward gain, p-controller
Kp = 1.0  # speed proportional gain
WB = 0.16  # [m] wheelbase of TurtleBot3 Burger
target_speed = 0.2  # [m/s] target speed

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.lookahead_distance = 1.5  # Lookahead distance for Pure Pursuit
        self.current_path = None
        self.current_index = 0
        self.state = State()
        self.goal_reached = False

    def path_callback(self, msg):
        self.current_path = msg
        self.current_index = 0
        self.goal_reached = False
        self.get_logger().info(f'Path received with {len(msg.poses)} waypoints.')

    def odom_callback(self, msg):
        self.state.update_from_odom(msg)

    def timer_callback(self):
        if self.current_path is None or self.goal_reached:
            return

        if self.current_index >= len(self.current_path.poses):
            self.stop_robot()
            self.get_logger().info('Goal reached. Stopping robot.')
            self.goal_reached = True
            return

        # Get the current target point from the path
        target_pose = self.current_path.poses[self.current_index].pose

        # Calculate the control inputs
        ai = proportional_control(target_speed, self.state.v)
        di, self.current_index = pure_pursuit_steer_control(self.state, self.current_path, self.current_index)

        # Update the state
        self.state.update(ai, di)

        # Create and publish the velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = self.state.v
        cmd_vel.angular.z = di
        self.cmd_vel_publisher.publish(cmd_vel)

        # Check if we are close enough to the target point
        if self.state.calc_distance(target_pose.position.x, target_pose.position.y) < self.lookahead_distance:
            self.current_index += 1

        self.get_logger().info(f'Current index: {self.current_index}, Total waypoints: {len(self.current_path.poses)}, Current position: ({self.state.x}, {self.state.y})')

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
    else:  # toward goal
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def search_target_index(state, cx, cy, pind):
    if pind is None:
        # search nearest point index
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

    # search look ahead target point index
    while Lf > state.calc_distance(cx[ind], cy[ind]):
        if (ind + 1) >= len(cx):
            break  # not exceed goal
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
'''

'''
#with /odom
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import math
import numpy as np

# Pure Pursuit Parameters
k = 0.1  # look forward gain
Lfc = 1.2  # [m] look-ahead distance (adjusted for TurtleBot3)
Kp = 1.0  # speed proportional gain
WB = 0.16  # [m] wheelbase of TurtleBot3 Burger
target_speed = 0.22  # [m/s] target speed

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.lookahead_distance = 1.2  # Lookahead distance for Pure Pursuit
        self.current_path = None
        self.current_index = 0
        self.state = State()

    def path_callback(self, msg):
        self.current_path = msg
        self.current_index = 0
        self.get_logger().info(f'Path received with {len(msg.poses)} waypoints.')

    def odom_callback(self, msg):
        self.state.update_from_odom(msg)

    def timer_callback(self):
        if self.current_path is None or self.current_index >= len(self.current_path.poses):
            return

        # Get the current target point from the path
        target_pose = self.current_path.poses[self.current_index].pose

        # Calculate the control inputs
        ai = proportional_control(target_speed, self.state.v)
        di, self.current_index = pure_pursuit_steer_control(self.state, self.current_path, self.current_index)

        # Update the state
        self.state.update(ai, di)

        # Create and publish the velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = self.state.v
        cmd_vel.angular.z = di
        self.cmd_vel_publisher.publish(cmd_vel)

        # Check if we are close enough to the target point
        if self.state.calc_distance(target_pose.position.x, target_pose.position.y) < self.lookahead_distance:
            self.current_index += 1

        self.get_logger().info(f'Current index: {self.current_index}, Total waypoints: {len(self.current_path.poses)}, Current position: ({self.state.x}, {self.state.y})')

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
    else:  # toward goal
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def search_target_index(state, cx, cy, pind):
    if pind is None:
        # search nearest point index
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

    # search look ahead target point index
    while Lf > state.calc_distance(cx[ind], cy[ind]):
        if (ind + 1) >= len(cx):
            break  # not exceed goal
        ind += 1

    return ind, Lf

def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
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
''' 
#without /odom
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import math
import numpy as np

# Pure Pursuit Parameters
k = 0.1  # look forward gain
Lfc = 0.1 #0.5  # [m] look-ahead distance (adjusted for TurtleBot3)
Kp = 1.0  # speed proportional gain
WB = 0.16  # [m] wheelbase of TurtleBot3 Burger
target_speed = 0.2  # [m/s] target speed

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.lookahead_distance = 0.1  # Lookahead distance for Pure Pursuit
        self.current_path = None
        self.current_index = 0
        self.state = State()

    def path_callback(self, msg):
        self.current_path = msg
        self.current_index = 0
        self.get_logger().info('Path received.')

    def timer_callback(self):
        if self.current_path is None or self.current_index >= len(self.current_path.poses):
            return

        # Get the current target point from the path
        target_pose = self.current_path.poses[self.current_index].pose

        # Calculate the control inputs
        ai = proportional_control(target_speed, self.state.v)
        di, self.current_index = pure_pursuit_steer_control(self.state, self.current_path, self.current_index)

        # Update the state
        self.state.update(ai, di)

        # Create and publish the velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = self.state.v
        cmd_vel.angular.z = di
        self.cmd_vel_publisher.publish(cmd_vel)

        # Check if we are close enough to the target point
        if self.state.calc_distance(target_pose.position.x, target_pose.position.y) < self.lookahead_distance:
            self.current_index += 1

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * 0.1
        self.y += self.v * math.sin(self.yaw) * 0.1
        self.yaw += self.v / WB * math.tan(delta) * 0.1
        self.v += a * 0.1

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)

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
    else:  # toward goal
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def search_target_index(state, cx, cy, pind):
    if pind is None:
        # search nearest point index
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

    # search look ahead target point index
    while Lf > state.calc_distance(cx[ind], cy[ind]):
        if (ind + 1) >= len(cx):
            break  # not exceed goal
        ind += 1

    return ind, Lf

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuitNode()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import math

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.lookahead_distance = 0.5  # Lookahead distance for Pure Pursuit
        self.current_path = None
        self.current_index = 0

    def path_callback(self, msg):
        self.current_path = msg
        self.current_index = 0
        self.get_logger().info('Path received.')

    def timer_callback(self):
        if self.current_path is None or self.current_index >= len(self.current_path.poses):
            return

        # Get the current target point from the path
        target_pose = self.current_path.poses[self.current_index].pose

        # Calculate the velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1  # Set the linear velocity

        # Calculate the heading angle
        target_angle = math.atan2(target_pose.position.y, target_pose.position.x)

        # Set the angular velocity based on the heading angle
        cmd_vel.angular.z = 1.0 * target_angle

        # Publish the velocity command
        self.cmd_vel_publisher.publish(cmd_vel)

        # Check if we are close enough to the target point
        if self.distance_to_target(target_pose.position) < self.lookahead_distance:
            self.current_index += 1

    def distance_to_target(self, target_position):
        # Assuming the robot starts at the origin (0,0)
        current_position = [0.0, 0.0]  # Replace with the actual current position if available
        return math.sqrt((target_position.x - current_position[0])**2 + (target_position.y - current_position[1])**2)

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuitNode()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


'''
#pure PurePursuit
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
import numpy as np

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.lookahead_distance = 0.1  # Adjust lookahead distance for TurtleBot3
        self.path = None
        self.current_pose = None

    def path_callback(self, msg):
        self.path = msg.poses

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.path is not None:
            self.pure_pursuit_control()

    def pure_pursuit_control(self):
        if self.current_pose is None or self.path is None:
            return

        current_position = np.array([self.current_pose.position.x, self.current_pose.position.y])
        lookahead_point = self.find_lookahead_point(current_position)
        if lookahead_point is None:
            return

        control_command = self.compute_control_command(current_position, lookahead_point)
        self.cmd_pub.publish(control_command)

    def find_lookahead_point(self, current_position):
        for pose in self.path:
            path_point = np.array([pose.pose.position.x, pose.pose.position.y])
            distance = np.linalg.norm(path_point - current_position)
            if distance > self.lookahead_distance:
                return path_point
        return None

    def compute_control_command(self, current_position, lookahead_point):
        control_command = Twist()
        robot_orientation = self.get_robot_orientation()
        angle_to_lookahead = np.arctan2(lookahead_point[1] - current_position[1], lookahead_point[0] - current_position[0])
        steering_angle = angle_to_lookahead - robot_orientation

        steering_angle = (steering_angle + np.pi) % (2 * np.pi) - np.pi

        control_command.linear.x = 0.2  # Adjust speed for TurtleBot3
        control_command.angular.z = 2.0 * steering_angle  # Adjust gain as necessary

        return control_command

    def get_robot_orientation(self):
        orientation_q = self.current_pose.orientation
        siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
