#'''
#sub /odom as a start point
#calcutate time and distacne generated path
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
import numpy as np
import random
import time

class RRTStarNode(Node):
    def __init__(self):
        super().__init__('rrt_star_planner')
        self.publisher_ = self.create_publisher(Marker, 'rrt_star_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        #self.create_subscription(Odometry, '/odom_estimate', self.odom_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.start = None
        self.goal = None
        self.D = 0.08
        self.max_iter = 100
        self.map = []
        self.parent = {}
        self.cost = {}
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.safety_distance = 0.25 #0.3
        self.radius = 0.3

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.LINE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.01
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
        self.initial_pose_set = False
        self.goal_received_time = None

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.get_logger().info('Map received.')

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

    def odom_callback(self, msg):
        if not self.initial_pose_set:
            self.start = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
            self.start_marker.pose.position.x = self.start[0]
            self.start_marker.pose.position.y = self.start[1]
            self.start_marker.pose.position.z = 0.0
            self.start_goal_publisher_.publish(self.start_marker)
            self.get_logger().info(f'Start position set to: {self.start}')
            self.reset_rrt()
            self.initial_pose_set = True

    def goalpose_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.goal_marker.pose.position.x = self.goal[0]
        self.goal_marker.pose.position.y = self.goal[1]
        self.goal_marker.pose.position.z = 0.0
        self.start_goal_publisher_.publish(self.goal_marker)
        self.get_logger().info(f'Goal position set to: {self.goal}')
        self.reset_rrt()
        self.goal_received_time = time.time()  # Start timing when the goal is received

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
                end_time = time.time()  # End timing when the path is found
                elapsed_time = (end_time - self.goal_received_time) * 1000  # Convert to milliseconds
                self.get_logger().info(f'Time taken to generate the path: {elapsed_time:.2f} ms')
                break

        self.publisher_.publish(self.marker)

    def publish_final_path(self, goal_point):
        planned_path = Path()
        planned_path.header.frame_id = "map"

        current_point = goal_point
        total_distance = 0.0
        previous_point = None
        while current_point is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = current_point[0]
            pose_stamped.pose.position.y = current_point[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            planned_path.poses.insert(0, pose_stamped)

            if previous_point is not None:
                total_distance += np.linalg.norm(np.array(previous_point) - np.array(current_point))

            previous_point = current_point
            current_point = self.parent.get(tuple(current_point), None)

        self.path_publisher_.publish(planned_path)
        
        # Assuming a constant velocity for simplicity
        constant_velocity = 0.4  # meters per second
        total_time = total_distance / constant_velocity
        
        self.get_logger().info(f'Total distance of the path: {total_distance:.2f} meters')
        self.get_logger().info(f'Estimated time to traverse the path: {total_time:.2f} seconds')

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt_star()
        else:
            self.get_logger().info('Monitoring environment for changes...')
        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    rrt_star_node = RRTStarNode()
    rclpy.spin(rrt_star_node)
    rrt_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



#'''


'''
#sub /odom as a start point
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
import numpy as np
import random

class RRTStarNode(Node):
    def __init__(self):
        super().__init__('rrt_star_planner')
        self.publisher_ = self.create_publisher(Marker, 'rrt_star_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom_estimate', self.odom_callback, 10)
        #self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.start = None
        self.goal = None
        self.D = 0.07
        self.max_iter = 100
        self.map = []
        self.parent = {}
        self.cost = {}
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.safety_distance = 0.105 #0.105 robot raduis
        #self.radius = 0.5
        self.radius = 0.3

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.LINE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.01
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
        self.initial_pose_set = False

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.get_logger().info('Map received.')

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

    def odom_callback(self, msg):
        if not self.initial_pose_set:
            self.start = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
            self.start_marker.pose.position.x = self.start[0]
            self.start_marker.pose.position.y = self.start[1]
            self.start_marker.pose.position.z = 0.0
            self.start_goal_publisher_.publish(self.start_marker)
            self.get_logger().info(f'Start position set to: {self.start}')
            self.reset_rrt()
            self.initial_pose_set = True

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
    rrt_star_node = RRTStarNode()
    rclpy.spin(rrt_star_node)
    rrt_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''



'''
#RRT*
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import random

class RRTStarNode(Node):
    def __init__(self):
        super().__init__('rrt_star_planner')
        self.publisher_ = self.create_publisher(Marker, 'rrt_star_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        #self.create_subscription(OccupancyGrid, '/global_map', self.map_callback, 10)
        #self.create_subscription(OccupancyGrid, '/local_map', self.map_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.start = None  # Initialize as None
        self.goal = None  # Initialize as None
        self.D = 0.2  # Good parameter 0.3 meters
        self.max_iter = 100  # Good 50 iterations
        self.map = []  # Start with an empty map
        self.parent = {}  # Initialize parent dictionary
        self.cost = {}  # Initialize cost dictionary
        self.map_data = None  # Initialize map data
        self.map_resolution = None
        self.map_origin = None
        self.safety_distance = 0.2  # Safety distance from obstacles
        self.radius = 0.5  # Radius for finding neighbors

        # Initialize marker for visualization
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.LINE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.02
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

        # Initialize markers for the start and goal points
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
        self.iterations = 0  # Initialize the counter
        self.max_iterations = 100  # Set the desired number of iterations

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.get_logger().info('Map received.')

    def is_in_obstacle(self, point):
        if self.map_data is None:
            return False

        pixel_x = int((point[0] - self.map_origin[0]) / self.map_resolution)
        pixel_y = int((point[1] - self.map_origin[1]) / self.map_resolution)

        # Check if the point is outside the map bounds
        if pixel_x < 0 or pixel_x >= self.map_data.shape[1] or pixel_y < 0 or pixel_y >= self.map_data.shape[0]:
            return True

        cell_value = self.map_data[pixel_y, pixel_x]
        if cell_value == 100:  # Occupied cell
            return True
        elif cell_value == 0:  # Free cell
            return False
        elif cell_value == -1:  # Unknown cell
            # Treat unknown as obstacle for safety
            return True
        else:
            # Handle unexpected values
            self.get_logger().warn(f'Unexpected cell value: {cell_value}')
            return True

    def is_near_obstacle(self, point):
        if self.map_data is None:
            return False

        pixel_x = int((point[0] - self.map_origin[0]) / self.map_resolution)
        pixel_y = int((point[1] - self.map_origin[1]) / self.map_resolution)

        # Define the search area around the point
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

            # Add points to the marker for visualization
            p1 = Point()
            p1.x, p1.y, p1.z = x_best[0], x_best[1], 0.0
            p2 = Point()
            p2.x, p2.y, p2.z = x_new[0], x_new[1], 0.0

            self.marker.points.append(p1)
            self.marker.points.append(p2)

            if np.linalg.norm(x_new - self.goal) < self.D:
                self.path_found = True
                self.get_logger().info('Path found to the goal.')
                self.publish_final_path(x_new)  # Call to publish final path
                break

        self.publisher_.publish(self.marker)

    def publish_final_path(self, goal_point):
        # Trace back from the goal to the start to get the path
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

            current_point = self.parent.get(tuple(current_point), None)  # Trace back using parent dictionary

        self.path_publisher_.publish(planned_path)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt_star()
        else:
            self.get_logger().info('Monitoring environment for changes...')
            # Add logic here to monitor environment and replan if necessary

        # Publish start and goal markers
        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    rrt_star_node = RRTStarNode()
    rclpy.spin(rrt_star_node)
    rrt_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''