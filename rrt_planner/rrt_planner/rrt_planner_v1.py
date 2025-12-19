import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
import numpy as np
import random
import math

class RRT:
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, start, goal, obstacle_list, rand_area, expand_dis=0.3, path_resolution=0.1, goal_sample_rate=5, max_iter=500, robot_radius=0.2):
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

    def planning(self, animation=False):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)

        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand), random.uniform(self.min_rand, self.max_rand))
        else:
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    @staticmethod
    def check_collision(node, obstacleList, robot_radius):
        if node is None:
            return False
        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]
            if min(d_list) <= (size + robot_radius)**2:
                return False
        return True

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

class RRTPlannerNode(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 5.0)
        self.declare_parameter('expand_dis', 0.3)
        self.declare_parameter('path_resolution', 0.1)
        self.declare_parameter('goal_sample_rate', 5)
        self.declare_parameter('max_iter', 500)
        self.declare_parameter('robot_radius', 0.2)
        self.declare_parameter('rand_area_min', -2.0)
        self.declare_parameter('rand_area_max', 10.0)

        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.expand_dis = self.get_parameter('expand_dis').value
        self.path_resolution = self.get_parameter('path_resolution').value
        self.goal_sample_rate = self.get_parameter('goal_sample_rate').value
        self.max_iter = self.get_parameter('max_iter').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.rand_area = [self.get_parameter('rand_area_min').value, self.get_parameter('rand_area_max').value]

        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_pose = None
        self.obstacle_list = []

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.plan_path()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def map_callback(self, msg):
        self.obstacle_list = []
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        for y in range(height):
            for x in range(width):
                if data[y, x] > 50:  # Assuming obstacle threshold
                    ox = origin_x + x * resolution
                    oy = origin_y + y * resolution
                    self.obstacle_list.append((ox, oy, resolution / 2))

    def plan_path(self):
        if self.current_pose is None:
            self.get_logger().info("Current pose is not available")
            return

        rrt = RRT(
            start=[self.current_pose.position.x, self.current_pose.position.y],
            goal=[self.goal_x, self.goal_y],
            rand_area=self.rand_area,
            expand_dis=self.expand_dis,
            path_resolution=self.path_resolution,
            goal_sample_rate=self.goal_sample_rate,
            max_iter=self.max_iter,
            robot_radius=self.robot_radius,
            obstacle_list=self.obstacle_list
        )
        path = rrt.planning(animation=False)

        if path is None:
            self.get_logger().info("Cannot find path")
        else:
            self.get_logger().info("Found path!")
            self.publish_path(path)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RRTPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
