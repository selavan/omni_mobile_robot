'''
#v3, rewire the entire tree
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import random

class MODRRTStarNode(Node):
    def __init__(self):
        super().__init__('mod_rrt_star_planner')
        self.publisher_ = self.create_publisher(Marker, 'rrt_star_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = None
        self.goal = None
        self.D = 0.2
        self.max_iter = 100
        self.map = set()
        self.parent = {}
        self.cost = {}
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.safety_distance = 0.3
        self.radius = 0.5

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.LINE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.02
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

        self.start_marker = Marker()
        self.start_marker.header.frame_id = "map"
        self.start_marker.type = Marker.SPHERE
        self.start_marker.action = Marker.ADD
        self.start_marker.scale.x = 0.2
        self.start_marker.scale.y = 0.2
        self.start_marker.scale.z = 0.2
        self.start_marker.color.a = 1.0
        self.start_marker.color.r = 0.0
        self.start_marker.color.g = 1.0
        self.start_marker.color.b = 0.0

        self.goal_marker = Marker()
        self.goal_marker.header.frame_id = "map"
        self.goal_marker.type = Marker.SPHERE
        self.goal_marker.action = Marker.ADD
        self.goal_marker.scale.x = 0.2
        self.goal_marker.scale.y = 0.2
        self.goal_marker.scale.z = 0.2
        self.goal_marker.color.a = 1.0
        self.goal_marker.color.r = 0.0
        self.goal_marker.color.g = 0.0
        self.goal_marker.color.b = 1.0

        self.path_found = False
        self.previous_map_data = None

    def map_callback(self, msg):
        self.previous_map_data = self.map_data.copy() if self.map_data is not None else None
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.get_logger().info('Map with static and dynamic obstacles received.')
        self.rewire_tree_due_to_new_obstacle()

    def is_in_obstacle(self, point):
        if self.map_data is None:
            return False

        pixel_x = int((point[0] - self.map_origin[0]) / self.map_resolution)
        pixel_y = int((point[1] - self.map_origin[1]) / self.map_resolution)

        if pixel_x < 0 or pixel_x >= self.map_data.shape[1] or pixel_y < 0 or pixel_y >= self.map_data.shape[0]:
            return True

        cell_value = self.map_data[pixel_y, pixel_x]
        if cell_value == 100:
            return True
        elif cell_value == 0:
            return False
        elif cell_value == -1:
            return True
        else:
            self.get_logger().warn(f'Unexpected cell value: {cell_value}')
            return True

    def is_near_obstacle(self, point):
        if self.map_data is None:
            return False

        pixel_x = int((point[0] - self.map_origin[0]) / self.map_resolution)
        pixel_y = int((point[1] - self.map_origin[1]) / self.map_resolution)

        search_radius = int(self.safety_distance / self.map_resolution)
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                nx = pixel_x + dx
                ny = pixel_y + dy
                if nx >= 0 and nx < self.map_data.shape[1] and ny >= 0 and ny < self.map_data.shape[0]:
                    if self.map_data[ny, nx] == 100:
                        return True
        return False

    def initialpose_callback(self, msg):
        self.start = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.start_marker.pose.position.x = self.start[0]
        self.start_marker.pose.position.y = self.start[1]
        self.start_marker.pose.position.z = 0.0
        self.start_goal_publisher_.publish(self.start_marker)
        self.get_logger().info(f'Start position set to: {self.start}')
        self.reset_rrt()

    def goalpose_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_marker.pose.position.x = self.goal[0]
        self.goal_marker.pose.position.y = self.goal[1]
        self.goal_marker.pose.position.z = 0.0
        self.start_goal_publisher_.publish(self.goal_marker)
        self.get_logger().info(f'Goal position set to: {self.goal}')
        self.reset_rrt()

    def reset_rrt(self):
        if self.start is not None and self.goal is not None:
            self.map = {self.start}
            self.parent = {self.start: None}
            self.cost = {self.start: 0}
            self.marker.points.clear()
            self.path_found = False    

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def find_neighbors(self, point):
        neighbors = []
        for p in self.map:
            if not self.is_in_obstacle(p) and np.linalg.norm(np.array(point) - np.array(p)) <= self.radius:
                neighbors.append(p)
        return neighbors

    def run_rrt_star(self):
        if self.start is None or self.goal is None:
            self.get_logger().info('Waiting for start and goal positions...')
            return
        if self.map_data is None:
            self.get_logger().info('Waiting for map data...')
            return

        for j in range(self.max_iter):
            x_rand = (
                random.uniform(self.map_origin[0], self.map_origin[0] + self.map_data.shape[1] * self.map_resolution),
                random.uniform(self.map_origin[1], self.map_origin[1] + self.map_data.shape[0] * self.map_resolution)
            )

            if self.is_in_obstacle(x_rand) or self.is_near_obstacle(x_rand):
                continue

            d_min = float('inf')
            x_near = None
            
            for point in self.map:
                d = np.linalg.norm(np.array(point) - np.array(x_rand))
                if d < d_min:
                    d_min = d
                    x_near = point

            if x_near is None:
                self.get_logger().warn('No near node found, skipping this iteration.')
                continue

            v = np.array(x_rand) - np.array(x_near)
            x_new = tuple(np.array(x_near) + (v / np.linalg.norm(v)) * self.D)

            if self.is_in_obstacle(x_new) or self.is_near_obstacle(x_new):
                continue

            x_neighbors = self.find_neighbors(x_new)
            x_best = x_near
            min_cost = self.cost[x_near] + np.linalg.norm(np.array(x_new) - np.array(x_near))

            for x_neighbor in x_neighbors:
                cost = self.cost[x_neighbor] + np.linalg.norm(np.array(x_new) - np.array(x_neighbor))
                if cost < min_cost:
                    x_best = x_neighbor
                    min_cost = cost

            self.map.add(x_new)
            self.parent[x_new] = x_best
            self.cost[x_new] = min_cost

            for x_neighbor in x_neighbors:
                new_cost = self.cost[x_new] + np.linalg.norm(np.array(x_new) - np.array(x_neighbor))
                if new_cost < self.cost[x_neighbor]:
                    self.parent[x_neighbor] = x_new
                    self.cost[x_neighbor] = new_cost

            p1 = Point()
            p1.x, p1.y, p1.z = x_best[0], x_best[1], 0.0
            p2 = Point()
            p2.x, p2.y, p2.z = x_new[0], x_new[1], 0.0

            self.marker.points.append(p1)
            self.marker.points.append(p2)

            if np.linalg.norm(np.array(x_new) - np.array(self.goal)) < self.D:
                self.path_found = True
                self.get_logger().info('Path found to the goal.')
                self.publish_final_path(x_new)
                break

        self.publisher_.publish(self.marker)

    def rewire_tree_due_to_new_obstacle(self):
        self.get_logger().info('Rewiring the entire tree due to new obstacle.')
        self.reset_rrt()
        self.run_rrt_star()

    def publish_final_path(self, goal_point):
        planned_path = Path()
        planned_path.header.frame_id = "map"

        current_point = goal_point
        while current_point is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = current_point[0]
            pose_stamped.pose.position.y = current_point[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            planned_path.poses.insert(0, pose_stamped)

            current_point = self.parent.get(current_point, None)

        self.path_publisher_.publish(planned_path)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt_star()
        else:
            self.get_logger().info('Monitoring environment for changes...')

        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    rrt_star_node = MODRRTStarNode()
    rclpy.spin(rrt_star_node)
    rrt_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''


#'''
#v2
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import random

class MODRRTStarNode(Node):
    def __init__(self):
        super().__init__('mod_rrt_star_planner')
        self.publisher_ = self.create_publisher(Marker, 'rrt_star_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.start = None
        self.goal = None
        self.D = 0.2
        self.max_iter = 120
        self.map = set()
        self.parent = {}
        self.cost = {}
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.safety_distance = 0.3
        self.radius = 0.5

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.LINE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.02
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

        self.start_marker = Marker()
        self.start_marker.header.frame_id = "map"
        self.start_marker.type = Marker.SPHERE
        self.start_marker.action = Marker.ADD
        self.start_marker.scale.x = 0.2
        self.start_marker.scale.y = 0.2
        self.start_marker.scale.z = 0.2
        self.start_marker.color.a = 1.0
        self.start_marker.color.r = 0.0
        self.start_marker.color.g = 1.0
        self.start_marker.color.b = 0.0

        self.goal_marker = Marker()
        self.goal_marker.header.frame_id = "map"
        self.goal_marker.type = Marker.SPHERE
        self.goal_marker.action = Marker.ADD
        self.goal_marker.scale.x = 0.2
        self.goal_marker.scale.y = 0.2
        self.goal_marker.scale.z = 0.2
        self.goal_marker.color.a = 1.0
        self.goal_marker.color.r = 0.0
        self.goal_marker.color.g = 0.0
        self.goal_marker.color.b = 1.0

        self.path_found = False
        self.previous_map_data = None

    def map_callback(self, msg):
        self.previous_map_data = self.map_data.copy() if self.map_data is not None else None
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.get_logger().info('Map with static and dynamic obstacles received.')
        self.path_found = False  # Trigger replanning

    def is_in_obstacle(self, point):
        if self.map_data is None:
            return False

        pixel_x = int((point[0] - self.map_origin[0]) / self.map_resolution)
        pixel_y = int((point[1] - self.map_origin[1]) / self.map_resolution)

        if pixel_x < 0 or pixel_x >= self.map_data.shape[1] or pixel_y < 0 or pixel_y >= self.map_data.shape[0]:
            return True

        cell_value = self.map_data[pixel_y, pixel_x]
        if cell_value == 100:
            return True
        elif cell_value == 0:
            return False
        elif cell_value == -1:
            return True
        else:
            self.get_logger().warn(f'Unexpected cell value: {cell_value}')
            return True

    def is_near_obstacle(self, point):
        if self.map_data is None:
            return False

        pixel_x = int((point[0] - self.map_origin[0]) / self.map_resolution)
        pixel_y = int((point[1] - self.map_origin[1]) / self.map_resolution)

        search_radius = int(self.safety_distance / self.map_resolution)
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                nx = pixel_x + dx
                ny = pixel_y + dy
                if nx >= 0 and nx < self.map_data.shape[1] and ny >= 0 and ny < self.map_data.shape[0]:
                    if self.map_data[ny, nx] == 100:
                        return True
        return False

    def initialpose_callback(self, msg):
        self.start = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.start_marker.pose.position.x = self.start[0]
        self.start_marker.pose.position.y = self.start[1]
        self.start_marker.pose.position.z = 0.0
        self.start_goal_publisher_.publish(self.start_marker)
        self.get_logger().info(f'Start position set to: {self.start}')
        self.reset_rrt()

    def goalpose_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_marker.pose.position.x = self.goal[0]
        self.goal_marker.pose.position.y = self.goal[1]
        self.goal_marker.pose.position.z = 0.0
        self.start_goal_publisher_.publish(self.goal_marker)
        self.get_logger().info(f'Goal position set to: {self.goal}')
        self.reset_rrt()

    def reset_rrt(self):
        if self.start is not None and self.goal is not None:
            self.map = {self.start}
            self.parent = {self.start: None}
            self.cost = {self.start: 0}
            self.marker.points.clear()
            self.path_found = False    

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def find_neighbors(self, point):
        neighbors = []
        for p in self.map:
            if np.linalg.norm(np.array(point) - np.array(p)) <= self.radius:
                neighbors.append(p)
        return neighbors

    def run_rrt_star(self):
        if self.start is None or self.goal is None:
            self.get_logger().info('Waiting for start and goal positions...')
            return
        if self.map_data is None:
            self.get_logger().info('Waiting for map data...')
            return

        if self.previous_map_data is not None and not np.array_equal(self.map_data, self.previous_map_data):
            self.rewire_tree_due_to_new_obstacle()

        for j in range(self.max_iter):
            x_rand = (
                random.uniform(self.map_origin[0], self.map_origin[0] + self.map_data.shape[1] * self.map_resolution),
                random.uniform(self.map_origin[1], self.map_origin[1] + self.map_data.shape[0] * self.map_resolution)
            )

            if self.is_in_obstacle(x_rand) or self.is_near_obstacle(x_rand):
                continue

            d_min = float('inf')
            x_near = None
            
            for point in self.map:
                d = np.linalg.norm(np.array(point) - np.array(x_rand))
                if d < d_min:
                    d_min = d
                    x_near = point

            v = np.array(x_rand) - np.array(x_near)
            x_new = tuple(np.array(x_near) + (v / np.linalg.norm(v)) * self.D)

            if self.is_in_obstacle(x_new) or self.is_near_obstacle(x_new):
                continue

            x_neighbors = self.find_neighbors(x_new)
            x_best = x_near
            min_cost = self.cost[x_near] + np.linalg.norm(np.array(x_new) - np.array(x_near))

            for x_neighbor in x_neighbors:
                cost = self.cost[x_neighbor] + np.linalg.norm(np.array(x_new) - np.array(x_neighbor))
                if cost < min_cost:
                    x_best = x_neighbor
                    min_cost = cost

            self.map.add(x_new)
            self.parent[x_new] = x_best
            self.cost[x_new] = min_cost

            for x_neighbor in x_neighbors:
                new_cost = self.cost[x_new] + np.linalg.norm(np.array(x_new) - np.array(x_neighbor))
                if new_cost < self.cost[x_neighbor]:
                    self.parent[x_neighbor] = x_new
                    self.cost[x_neighbor] = new_cost

            p1 = Point()
            p1.x, p1.y, p1.z = x_best[0], x_best[1], 0.0
            p2 = Point()
            p2.x, p2.y, p2.z = x_new[0], x_new[1], 0.0

            self.marker.points.append(p1)
            self.marker.points.append(p2)

            if np.linalg.norm(np.array(x_new) - np.array(self.goal)) < self.D:
                self.path_found = True
                self.get_logger().info('Path found to the goal.')
                self.publish_final_path(x_new)
                break

        self.publisher_.publish(self.marker)

    def rewire_tree_due_to_new_obstacle(self):
        # Remove nodes and edges affected by the new obstacle
        nodes_to_remove = {node for node in self.map if self.is_in_obstacle(node) or self.is_near_obstacle(node)}

        for node in nodes_to_remove:
            self.map.remove(node)
            if node in self.parent:
                del self.parent[node]
            if node in self.cost:
                del self.cost[node]

        # Rewire the tree to maintain connectivity and optimality
        for node in self.map:
            neighbors = self.find_neighbors(node)
            for neighbor in neighbors:
                new_cost = self.cost[node] + np.linalg.norm(np.array(node) - np.array(neighbor))
                if new_cost < self.cost.get(neighbor, float('inf')):
                    self.parent[neighbor] = node
                    self.cost[neighbor] = new_cost

    def publish_final_path(self, goal_point):
        planned_path = Path()
        planned_path.header.frame_id = "map"

        current_point = goal_point
        while current_point is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = current_point[0]
            pose_stamped.pose.position.y = current_point[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            planned_path.poses.insert(0, pose_stamped)

            current_point = self.parent.get(current_point, None)

        self.path_publisher_.publish(planned_path)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt_star()
        else:
            self.get_logger().info('Monitoring environment for changes...')

        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    rrt_star_node = MODRRTStarNode()
    rclpy.spin(rrt_star_node)
    rrt_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#'''

'''
#v1
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import random

class MODRRTStarNode(Node):
    def __init__(self):
        super().__init__('mod_rrt_star_planner')
        self.publisher_ = self.create_publisher(Marker, 'rrt_star_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = None
        self.goal = None
        self.D = 0.2
        self.max_iter = 100
        self.map = []
        self.parent = {}
        self.cost = {}
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.safety_distance = 0.3
        self.radius = 0.5

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.LINE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.02
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

        self.start_marker = Marker()
        self.start_marker.header.frame_id = "map"
        self.start_marker.type = Marker.SPHERE
        self.start_marker.action = Marker.ADD
        self.start_marker.scale.x = 0.2
        self.start_marker.scale.y = 0.2
        self.start_marker.scale.z = 0.2
        self.start_marker.color.a = 1.0
        self.start_marker.color.r = 0.0
        self.start_marker.color.g = 1.0
        self.start_marker.color.b = 0.0

        self.goal_marker = Marker()
        self.goal_marker.header.frame_id = "map"
        self.goal_marker.type = Marker.SPHERE
        self.goal_marker.action = Marker.ADD
        self.goal_marker.scale.x = 0.2
        self.goal_marker.scale.y = 0.2
        self.goal_marker.scale.z = 0.2
        self.goal_marker.color.a = 1.0
        self.goal_marker.color.r = 0.0
        self.goal_marker.color.g = 0.0
        self.goal_marker.color.b = 1.0

        self.path_found = False

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.get_logger().info('Map with static and dynamic obstacles received.')
        self.path_found = False  # Trigger replanning

    def is_in_obstacle(self, point):
        if self.map_data is None:
            return False

        pixel_x = int((point[0] - self.map_origin[0]) / self.map_resolution)
        pixel_y = int((point[1] - self.map_origin[1]) / self.map_resolution)

        if pixel_x < 0 or pixel_x >= self.map_data.shape[1] or pixel_y < 0 or pixel_y >= self.map_data.shape[0]:
            return True

        cell_value = self.map_data[pixel_y, pixel_x]
        if cell_value == 100:
            return True
        elif cell_value == 0:
            return False
        elif cell_value == -1:
            return True
        else:
            self.get_logger().warn(f'Unexpected cell value: {cell_value}')
            return True

    def is_near_obstacle(self, point):
        if self.map_data is None:
            return False

        pixel_x = int((point[0] - self.map_origin[0]) / self.map_resolution)
        pixel_y = int((point[1] - self.map_origin[1]) / self.map_resolution)

        search_radius = int(self.safety_distance / self.map_resolution)
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                nx = pixel_x + dx
                ny = pixel_y + dy
                if nx >= 0 and nx < self.map_data.shape[1] and ny >= 0 and ny < self.map_data.shape[0]:
                    if self.map_data[ny, nx] == 100:
                        return True
        return False

    def initialpose_callback(self, msg):
        self.start = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.start_marker.pose.position.x = self.start[0]
        self.start_marker.pose.position.y = self.start[1]
        self.start_marker.pose.position.z = 0.0
        self.start_goal_publisher_.publish(self.start_marker)
        self.get_logger().info(f'Start position set to: {self.start}')
        self.reset_rrt()

    def goalpose_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.goal_marker.pose.position.x = self.goal[0]
        self.goal_marker.pose.position.y = self.goal[1]
        self.goal_marker.pose.position.z = 0.0
        self.start_goal_publisher_.publish(self.goal_marker)
        self.get_logger().info(f'Goal position set to: {self.goal}')
        self.reset_rrt()

    def reset_rrt(self):
        if self.start is not None and self.goal is not None:
            self.map = [self.start]
            self.parent = {tuple(self.start): None}
            self.cost = {tuple(self.start): 0}
            self.marker.points.clear()
            self.path_found = False    

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def find_neighbors(self, point):
        neighbors = []
        for p in self.map:
            if np.linalg.norm(point - p) <= self.radius:
                neighbors.append(p)
        return neighbors

    def run_rrt_star(self):
        if self.start is None or self.goal is None:
            self.get_logger().info('Waiting for start and goal positions...')
            return
        if self.map_data is None:
            self.get_logger().info('Waiting for map data...')
            return

        for j in range(self.max_iter):
            x_rand = np.array([
                random.uniform(self.map_origin[0], self.map_origin[0] + self.map_data.shape[1] * self.map_resolution),
                random.uniform(self.map_origin[1], self.map_origin[1] + self.map_data.shape[0] * self.map_resolution)
            ])
            
            if self.is_in_obstacle(x_rand) or self.is_near_obstacle(x_rand):
                continue

            d_min = float('inf')
            i_min = 0
            
            for i, point in enumerate(self.map):
                d = np.linalg.norm(point - x_rand)
                if d < d_min:
                    d_min = d
                    i_min = i

            x_near = self.map[i_min]
            v = x_rand - x_near
            x_new = x_near + (v / np.linalg.norm(v)) * self.D

            if self.is_in_obstacle(x_new) or self.is_near_obstacle(x_new):
                continue

            x_neighbors = self.find_neighbors(x_new)
            x_best = x_near
            min_cost = self.cost[tuple(x_near)] + np.linalg.norm(x_new - x_near)

            for x_neighbor in x_neighbors:
                cost = self.cost[tuple(x_neighbor)] + np.linalg.norm(x_new - x_neighbor)
                if cost < min_cost:
                    x_best = x_neighbor
                    min_cost = cost

            self.map.append(x_new)
            self.parent[tuple(x_new)] = tuple(x_best)
            self.cost[tuple(x_new)] = min_cost

            for x_neighbor in x_neighbors:
                new_cost = self.cost[tuple(x_new)] + np.linalg.norm(x_new - x_neighbor)
                if new_cost < self.cost[tuple(x_neighbor)]:
                    self.parent[tuple(x_neighbor)] = tuple(x_new)
                    self.cost[tuple(x_neighbor)] = new_cost

            p1 = Point()
            p1.x, p1.y, p1.z = x_best[0], x_best[1], 0.0
            p2 = Point()
            p2.x, p2.y, p2.z = x_new[0], x_new[1], 0.0

            self.marker.points.append(p1)
            self.marker.points.append(p2)

            if np.linalg.norm(x_new - self.goal) < self.D:
                self.path_found = True
                self.get_logger().info('Path found to the goal.')
                self.publish_final_path(x_new)
                break

        self.publisher_.publish(self.marker)

    def publish_final_path(self, goal_point):
        planned_path = Path()
        planned_path.header.frame_id = "map"

        current_point = goal_point
        while current_point is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = current_point[0]
            pose_stamped.pose.position.y = current_point[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            planned_path.poses.insert(0, pose_stamped)

            current_point = self.parent.get(tuple(current_point), None)

        self.path_publisher_.publish(planned_path)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt_star()
        else:
            self.get_logger().info('Monitoring environment for changes...')

        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    rrt_star_node = MODRRTStarNode()
    rclpy.spin(rrt_star_node)
    rrt_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

'''
#pure MOD-RRT*
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Pose
from visualization_msgs.msg import Marker
import numpy as np
import random
import math

class MODRRTStarNode(Node):
    def __init__(self):
        super().__init__('mod_rrt_star_node')
        
        self.publisher_ = self.create_publisher(Marker, 'rrt_star_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Path, 'planned_path', 10)
        
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        self.map_data = None
        self.start_pos = None
        self.goal_pos = None
        self.state_tree = {'vertices': [], 'edges': []}

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info('Map received')

    def initialpose_callback(self, msg):
        self.start_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.get_logger().info('Start position updated')

    def goalpose_callback(self, msg):
        self.goal_pos = [msg.pose.position.x, msg.pose.position.y]
        self.get_logger().info('Goal position set')
        if self.map_data and self.start_pos and self.goal_pos:
            self.generate_initial_path()

    def generate_initial_path(self):
        self.state_tree['vertices'] = [self.goal_pos]
        self.state_tree['edges'] = []
        while not self.tree_contains(self.start_pos):
            x_rand = self.random_sample()
            x_nearest = self.nearest(self.state_tree['vertices'], x_rand)
            x_new = self.steer(x_nearest, x_rand)
            if self.collision_free(x_nearest, x_new):
                x_parent = x_nearest
                self.state_tree['vertices'].append(x_new)
                self.state_tree['edges'].append((x_parent, x_new))
                self.rewire(x_new)
        self.optimize_path()
        self.publish_path()

    def random_sample(self):
        # Randomly sample within the map bounds
        x = random.uniform(0, self.map_data.info.width * self.map_data.info.resolution)
        y = random.uniform(0, self.map_data.info.height * self.map_data.info.resolution)
        return [x, y]

    def nearest(self, vertices, x_rand):
        # Find the nearest vertex in the tree to the random sample
        return min(vertices, key=lambda v: np.linalg.norm(np.array(v) - np.array(x_rand)))

    def steer(self, x_nearest, x_rand, eta=0.5):
        # Steer towards the random sample from the nearest vertex
        direction = np.array(x_rand) - np.array(x_nearest)
        length = np.linalg.norm(direction)
        direction = direction / length
        length = min(length, eta)
        return (np.array(x_nearest) + length * direction).tolist()

    def collision_free(self, x1, x2):
        # Check if the path between x1 and x2 is collision-free
        return True  # Simplified for brevity; implement actual collision checking

    def rewire(self, x_new):
        # Rewire the tree to maintain optimality
        for x_near in self.neighbors(x_new):
            if self.collision_free(x_new, x_near) and self.cost(x_new) + self.distance(x_new, x_near) < self.cost(x_near):
                self.state_tree['edges'].remove((self.parent(x_near), x_near))
                self.state_tree['edges'].append((x_new, x_near))

    def optimize_path(self):
        # Optimize the initial path for smoothness and length
        pass  # Implement path optimization

    def publish_path(self):
        # Publish the planned path
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for vertex in self.state_tree['vertices']:
            pose = PoseStamped()
            pose.pose.position = Point(x=vertex[0], y=vertex[1], z=0.0)
            path_msg.poses.append(pose)
        self.path_publisher_.publish(path_msg)

    def neighbors(self, x_new, radius=1.0):
        # Find neighbors within a certain radius
        return [v for v in self.state_tree['vertices'] if np.linalg.norm(np.array(v) - np.array(x_new)) < radius]

    def cost(self, x):
        # Calculate the cost of reaching x from the start
        return np.linalg.norm(np.array(x) - np.array(self.start_pos))

    def distance(self, x1, x2):
        # Calculate the Euclidean distance between two points
        return np.linalg.norm(np.array(x1) - np.array(x2))

    def parent(self, x):
        # Find the parent of a vertex in the tree
        for edge in self.state_tree['edges']:
            if edge[1] == x:
                return edge[0]
        return None

    def tree_contains(self, x):
        # Check if the tree contains a vertex close to x
        return any(np.linalg.norm(np.array(v) - np.array(x)) < 0.5 for v in self.state_tree['vertices'])

def main(args=None):
    rclpy.init(args=args)
    node = MODRRTStarNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''