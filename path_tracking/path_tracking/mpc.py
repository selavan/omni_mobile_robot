import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import math
import numpy as np
import cvxpy

# Vehicle parameters
LENGTH = 0.138  # [m]
WIDTH = 0.16  # [m]
WB = 0.16  # [m]

# MPC parameters
NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 10  # horizon length

R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 0.1  # goal distance
STOP_SPEED = 0.01  # stop speed
MAX_TIME = 500.0  # max simulation time

# Iterative parameter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 0.15  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.1  # [s] time tick

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 0.22  # maximum speed [m/s]
MIN_SPEED = -0.22  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/s²]

show_animation = True


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * DT
        self.y += self.v * math.sin(self.yaw) * DT
        self.yaw += self.v / WB * math.tan(delta) * DT
        self.v += a * DT

        if self.v > MAX_SPEED:
            self.v = MAX_SPEED
        elif self.v < MIN_SPEED:
            self.v = MIN_SPEED

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)

    def update_from_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = self.euler_from_quaternion(orientation)
        self.v = msg.twist.twist.linear.x

    def euler_from_quaternion(self, quaternion):
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


def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def get_linear_model_matrix(v, phi, delta):
    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = -DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = -DT * v * math.cos(phi) * phi
    C[3] = -DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C


class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

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

        # Calculate the reference trajectory
        xref, dref = self.calc_ref_trajectory()

        # Perform MPC control
        oa, odelta = self.iterative_linear_mpc_control(xref, dref)

        if oa is not None and odelta is not None:
            # Update the state
            self.state.update(oa[0], odelta[0])

            # Create and publish the velocity command
            cmd_vel = Twist()
            cmd_vel.linear.x = self.state.v
            cmd_vel.angular.z = odelta[0]
            self.cmd_vel_publisher.publish(cmd_vel)

        # Check if we are close enough to the target point
        if self.state.calc_distance(target_pose.position.x, target_pose.position.y) < self.goal_tolerance:
            self.current_index += 1

        self.get_logger().info(f'Current index: {self.current_index}, Total waypoints: {len(self.current_path.poses)}, Current position: ({self.state.x}, {self.state.y})')

    def calc_ref_trajectory(self):
        xref = np.zeros((NX, T + 1))
        dref = np.zeros((1, T + 1))
        ncourse = len(self.current_path.poses)

        ind, _ = self.calc_nearest_index()

        if self.current_index >= ind:
            ind = self.current_index

        for i in range(T + 1):
            if (ind + i) < ncourse:
                pose = self.current_path.poses[ind + i].pose
                xref[0, i] = pose.position.x
                xref[1, i] = pose.position.y
                xref[2, i] = TARGET_SPEED
                xref[3, i] = self.get_yaw_from_pose(pose)
                dref[0, i] = 0.0
            else:
                pose = self.current_path.poses[-1].pose
                xref[0, i] = pose.position.x
                xref[1, i] = pose.position.y
                xref[2, i] = TARGET_SPEED
                xref[3, i] = self.get_yaw_from_pose(pose)
                dref[0, i] = 0.0

        return xref, dref

    def iterative_linear_mpc_control(self, xref, dref):
        x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]  # current state
        oa, odelta = [0.0] * T, [0.0] * T  # Initialize control inputs

        for _ in range(MAX_ITER):
            xbar = self.predict_motion(x0, oa, odelta, xref)
            poa, pod = oa[:], odelta[:]
            oa, odelta, _, _, _, _ = self.linear_mpc_control(xref, xbar, x0, dref)
            if oa is None or odelta is None:
                self.get_logger().error("MPC solver failed to find a solution.")
                return None, None
            du = sum(abs(oa - poa)) + sum(abs(odelta - pod))  # calc u change value
            if du <= DU_TH:
                break
        else:
            self.get_logger().info("Iterative is max iter")

        return oa, odelta

    def linear_mpc_control(self, xref, xbar, x0, dref):
        x = cvxpy.Variable((NX, T + 1))
        u = cvxpy.Variable((NU, T))

        cost = 0.0
        constraints = []

        for t in range(T):
            cost += cvxpy.quad_form(u[:, t], R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

            A, B, C = get_linear_model_matrix(xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= MAX_DSTEER * DT]

        cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= MAX_SPEED]
        constraints += [x[2, :] >= MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.ECOS, verbose=True)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = np.array(x.value[0, :]).flatten()
            oy = np.array(x.value[1, :]).flatten()
            ov = np.array(x.value[2, :]).flatten()
            oyaw = np.array(x.value[3, :]).flatten()
            oa = np.array(u.value[0, :]).flatten()
            odelta = np.array(u.value[1, :]).flatten()
        else:
            self.get_logger().error("Error: Cannot solve mpc..")
            return None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov

    def predict_motion(self, x0, oa, odelta, xref):
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, odelta, range(1, T + 1)):
            state.update(ai, di)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar

    def calc_nearest_index(self):
        dx = [self.state.x - pose.pose.position.x for pose in self.current_path.poses[self.current_index:(self.current_index + N_IND_SEARCH)]]
        dy = [self.state.y - pose.pose.position.y for pose in self.current_path.poses[self.current_index:(self.current_index + N_IND_SEARCH)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + self.current_index

        mind = math.sqrt(mind)

        dxl = self.current_path.poses[ind].pose.position.x - self.state.x
        dyl = self.current_path.poses[ind].pose.position.y - self.state.y

        angle = pi_2_pi(self.get_yaw_from_pose(self.current_path.poses[ind].pose) - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind

    def get_yaw_from_pose(self, pose):
        orientation = pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation)
        return yaw

    def euler_from_quaternion(self, quaternion):
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
            ai = pd_control(distance_to_goal, self.state.v, 0, 0)

            # Proportional control for angular speed to face the goal
            angle_to_goal = math.atan2(self.goal_pose.position.y - self.state.y, self.goal_pose.position.x - self.state.x) - self.state.yaw
            di = math.atan2(2.0 * WB * math.sin(angle_to_goal) / self.goal_tolerance, 1.0)

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


def pd_control(target, current, target_derivative, current_derivative):
    return Kp * (target - current) + Kd * (target_derivative - current_derivative)


def main(args=None):
    rclpy.init(args=args)
    mpc_node = MPCNode()
    rclpy.spin(mpc_node)
    mpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
