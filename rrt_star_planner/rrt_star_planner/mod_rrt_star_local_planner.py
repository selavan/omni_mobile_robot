#'''
#map with dynamics and static, updated v1
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
        self.publisher_ = self.create_publisher(Marker, 'mod_rrt_star_path', 10)
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

        # Initialize tree attributes
        self.map = []
        self.parent = {}
        self.cost = {}

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
        self.replan_path()

    def goalpose_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.goal_marker.pose.position.x = self.goal[0]
        self.goal_marker.pose.position.y = self.goal[1]
        self.goal_marker.pose.position.z = 0.0
        self.start_goal_publisher_.publish(self.goal_marker)
        self.get_logger().info(f'Goal position set to: {self.goal}')
        self.replan_path()

    def replan_path(self):
        if self.start is None or self.goal is None:
            self.get_logger().info("Start or goal is not set. Replanning aborted.")
            return
        self.reset_tree()
        self.run_rrt_star()

    def reset_tree(self):
        self.map = [self.start]
        self.parent = {tuple(self.start): None}
        self.cost = {tuple(self.start): 0}
        self.marker.points.clear()
        self.path_found = False

    def run_rrt_star(self):
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
            i_min = -1

            for i, point in enumerate(self.map):
                d = np.linalg.norm(point - x_rand)
                if d < d_min:
                    d_min = d
                    i_min = i

            if i_min == -1:
                self.get_logger().error(f'No valid nearest node found for x_rand: {x_rand}')
                continue

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

            # Rewire the tree
            self.rewire_tree(x_new, x_neighbors)

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

    def find_neighbors(self, point):
        neighbors = []
        for p in self.map:
            if np.linalg.norm(point - p) <= self.radius:
                neighbors.append(p)
        return neighbors

    def rewire_tree(self, x_new, x_neighbors):
        for x_neighbor in x_neighbors:
            new_cost = self.cost[tuple(x_new)] + np.linalg.norm(x_new - x_neighbor)
            if new_cost < self.cost[tuple(x_neighbor)]:
                if not self.is_in_obstacle(x_neighbor):
                    self.parent[tuple(x_neighbor)] = tuple(x_new)
                    self.cost[tuple(x_neighbor)] = new_cost

                    p1 = Point()
                    p1.x, p1.y, p1.z = x_new[0], x_new[1], 0.0
                    p2 = Point()
                    p2.x, p2.y, p2.z = x_neighbor[0], x_neighbor[1], 0.0

                    self.marker.points.append(p1)
                    self.marker.points.append(p2)

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt_star()
        else:
            self.get_logger().info('Monitoring environment for changes...')
            self.replan_path()  # Trigger replanning if necessary

        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    mod_rrt_star_node = MODRRTStarNode()
    rclpy.spin(mod_rrt_star_node)
    mod_rrt_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#'''


'''
#map with dynamics and static, updated
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
        self.publisher_ = self.create_publisher(Marker, 'mod_rrt_star_path', 10)
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

        # Initialize tree attributes
        self.map = []
        self.parent = {}
        self.cost = {}

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
        self.replan_path()

    def goalpose_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.goal_marker.pose.position.x = self.goal[0]
        self.goal_marker.pose.position.y = self.goal[1]
        self.goal_marker.pose.position.z = 0.0
        self.start_goal_publisher_.publish(self.goal_marker)
        self.get_logger().info(f'Goal position set to: {self.goal}')
        self.replan_path()

    def replan_path(self):
        if self.start is None or self.goal is None:
            self.get_logger().info("Start or goal is not set. Replanning aborted.")
            return
        self.reset_tree()
        self.run_rrt_star()

    def reset_tree(self):
        self.map = [self.start]
        self.parent = {tuple(self.start): None}
        self.cost = {tuple(self.start): 0}
        self.marker.points.clear()
        self.path_found = False

    def run_rrt_star(self):
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
            i_min = -1

            for i, point in enumerate(self.map):
                d = np.linalg.norm(point - x_rand)
                if d < d_min:
                    d_min = d
                    i_min = i

            if i_min == -1:
                self.get_logger().error(f'No valid nearest node found for x_rand: {x_rand}')
                continue

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

    def find_neighbors(self, point):
        neighbors = []
        for p in self.map:
            if np.linalg.norm(point - p) <= self.radius:
                neighbors.append(p)
        return neighbors

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt_star()
        else:
            self.get_logger().info('Monitoring environment for changes...')

        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    mod_rrt_star_node = MODRRTStarNode()
    rclpy.spin(mod_rrt_star_node)
    mod_rrt_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''

'''
#map with dynamics and static
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
        self.publisher_ = self.create_publisher(Marker, 'mod_rrt_star_path', 10)
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

        # Initialize tree attributes
        self.map = []
        self.parent = {}
        self.cost = {}

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
        self.replan_path()

    def goalpose_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.goal_marker.pose.position.x = self.goal[0]
        self.goal_marker.pose.position.y = self.goal[1]
        self.goal_marker.pose.position.z = 0.0
        self.start_goal_publisher_.publish(self.goal_marker)
        self.get_logger().info(f'Goal position set to: {self.goal}')
        self.replan_path()

    def replan_path(self):
        if self.start is None or self.goal is None:
            return
        self.reset_tree()
        self.run_rrt_star()

    def reset_tree(self):
        self.map = [self.start]
        self.parent = {tuple(self.start): None}
        self.cost = {tuple(self.start): 0}
        self.marker.points.clear()
        self.path_found = False

    def run_rrt_star(self):
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
            i_min = -1

            for i, point in enumerate(self.map):
                d = np.linalg.norm(point - x_rand)
                if d < d_min:
                    d_min = d
                    i_min = i

            if i_min == -1:
                self.get_logger().error('No valid nearest node found.')
                continue

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

    def find_neighbors(self, point):
        neighbors = []
        for p in self.map:
            if np.linalg.norm(point - p) <= self.radius:
                neighbors.append(p)
        return neighbors

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt_star()
        else:
            self.get_logger().info('Monitoring environment for changes...')

        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    mod_rrt_star_node = MODRRTStarNode()
    rclpy.spin(mod_rrt_star_node)
    mod_rrt_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''


'''
#global and local map
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
        self.publisher_ = self.create_publisher(Marker, 'mod_rrt_star_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.create_subscription(OccupancyGrid, '/global_map', self.global_map_callback, 10)
        self.create_subscription(OccupancyGrid, '/local_map', self.local_map_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = None
        self.goal = None
        self.D = 0.2
        self.max_iter = 100
        self.global_map_data = None
        self.local_map_data = None
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
        self.iterations = 0
        self.max_iterations = 100

        # Initialize tree attributes
        self.map = []
        self.parent = {}
        self.cost = {}

    def global_map_callback(self, msg):
        self.global_map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.get_logger().info('Global map received.')

    def local_map_callback(self, msg):
        self.local_map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.get_logger().info('Local map received. Triggering replanning.')
        self.replan_path()

    def initialpose_callback(self, msg):
        self.start = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.start_marker.pose.position.x = self.start[0]
        self.start_marker.pose.position.y = self.start[1]
        self.start_goal_publisher_.publish(self.start_marker)
        self.get_logger().info(f'Start position set to: {self.start}')
        self.replan_path()

    def goalpose_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.goal_marker.pose.position.x = self.goal[0]
        self.goal_marker.pose.position.y = self.goal[1]
        self.start_goal_publisher_.publish(self.goal_marker)
        self.get_logger().info(f'Goal position set to: {self.goal}')
        self.replan_path()

    def replan_path(self):
        if self.start is None or self.goal is None:
            return
        self.reset_tree()
        self.run_rrt_star()

    def reset_tree(self):
        self.map = [self.start]
        self.parent = {tuple(self.start): None}
        self.cost = {tuple(self.start): 0}
        self.marker.points.clear()
        self.path_found = False

    def run_rrt_star(self):
        if self.global_map_data is None:
            self.get_logger().info('Waiting for global map data...')
            return

        map_data = self.global_map_data.copy()
        if self.local_map_data is not None:
            map_data = np.maximum(map_data, self.local_map_data)

        for j in range(self.max_iter):
            x_rand = np.array([
                random.uniform(self.map_origin[0], self.map_origin[0] + map_data.shape[1] * self.map_resolution),
                random.uniform(self.map_origin[1], self.map_origin[1] + map_data.shape[0] * self.map_resolution)
            ])
            
            if self.is_in_obstacle(x_rand, map_data):
                continue

            d_min = float('inf')
            i_min = -1

            for i, point in enumerate(self.map):
                d = np.linalg.norm(point - x_rand)
                if d < d_min:
                    d_min = d
                    i_min = i

            if i_min == -1:
                self.get_logger().error('No valid nearest node found.')
                continue

            x_near = self.map[i_min]
            v = x_rand - x_near
            x_new = x_near + (v / np.linalg.norm(v)) * self.D

            if self.is_in_obstacle(x_new, map_data):
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

    def is_in_obstacle(self, point, map_data):
        if map_data is None:
            return False

        pixel_x = int((point[0] - self.map_origin[0]) / self.map_resolution)
        pixel_y = int((point[1] - self.map_origin[1]) / self.map_resolution)

        if pixel_x < 0 or pixel_x >= map_data.shape[1] or pixel_y < 0 or pixel_y >= map_data.shape[0]:
            return True

        cell_value = map_data[pixel_y, pixel_x]
        return cell_value == 100

    def find_neighbors(self, point):
        neighbors = []
        for p in self.map:
            if np.linalg.norm(point - p) <= self.radius:
                neighbors.append(p)
        return neighbors

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt_star()
        else:
            self.get_logger().info('Monitoring environment for changes...')

        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    mod_rrt_star_node = MODRRTStarNode()
    rclpy.spin(mod_rrt_star_node)
    mod_rrt_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''