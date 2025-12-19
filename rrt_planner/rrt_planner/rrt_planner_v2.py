#'''
#final path
#2D Pose Estimate as a start point
#2D Goal Pose as a goal point
#Occupancy grid map, from slam toolbox
#Publish /planned_path, subscribe by Pure Pursuit
#Add safety distance from obstacles
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import random

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        self.publisher_ = self.create_publisher(Marker, 'rrt_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Path, 'planned_path', 10)  # Changed to publish Path
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = None  # Initialize as None
        self.goal = None  # Initialize as None
        self.D = 0.2  # Good parameter 0.3 meters
        self.max_iter = 100  # Good 50 iterations
        self.map = []  # Start with an empty map
        self.parent = {}  # Initialize parent dictionary
        self.map_data = None  # Initialize map data
        self.map_resolution = None
        self.map_origin = None
        self.safety_distance = 0.1  # Safety distance from obstacles

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
            self.marker.points.clear()
            self.path_found = False    

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def run_rrt(self):
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

            self.map.append(x_new)
            self.parent[tuple(x_new)] = tuple(x_near)  # Update parent dictionary

            # Add points to the marker for visualization
            p1 = Point()
            p1.x, p1.y, p1.z = x_near[0], x_near[1], 0.0
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
            self.run_rrt()
        else:
            self.get_logger().info('Monitoring environment for changes...')
            # Add logic here to monitor environment and replan if necessary

        # Publish start and goal markers
        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#'''
'''
#final path
#2D Pose Estimate as a start point
#2D Goal Pose as a goal point
#Occupancy grid map, from slam toolbox
#Publish /planned_path, subscribe by Pure Pursuit
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import random

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        self.publisher_ = self.create_publisher(Marker, 'rrt_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Path, 'planned_path', 10)  # Changed to publish Path
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = None  # Initialize as None
        self.goal = None  # Initialize as None
        self.D = 0.2  # Good parameter 0.3 meters
        self.max_iter = 100  # Good 50 iterations
        self.map = []  # Start with an empty map
        self.parent = {}  # Initialize parent dictionary
        self.map_data = None  # Initialize map data
        self.map_resolution = None
        self.map_origin = None

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
            self.marker.points.clear()
            self.path_found = False    

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def run_rrt(self):
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
            
            if self.is_in_obstacle(x_rand):
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

            if self.is_in_obstacle(x_new):
                continue

            self.map.append(x_new)
            self.parent[tuple(x_new)] = tuple(x_near)  # Update parent dictionary

            # Add points to the marker for visualization
            p1 = Point()
            p1.x, p1.y, p1.z = x_near[0], x_near[1], 0.0
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
            self.run_rrt()
        else:
            self.get_logger().info('Monitoring environment for changes...')
            # Add logic here to monitor environment and replan if necessary

        # Publish start and goal markers
        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
#final path
#2D Pose Estimate as a start point
#2D Goal Pose as a goal point
#Occupancy grid map, from slam toolbox
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import random

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.publisher_ = self.create_publisher(Marker, 'rrt_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Marker, 'final_path', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = None  # Initialize as None
        self.goal = None  # Initialize as None
        self.D = 0.1  # Good parameter 0.3 meters
        self.max_iter = 50  # Good 50 iterations
        self.map = []  # Start with an empty map
        self.parent = {}  # Initialize parent dictionary
        self.map_data = None  # Initialize map data
        self.map_resolution = None
        self.map_origin = None

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
            self.marker.points.clear()
            self.path_found = False    

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def run_rrt(self):
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
            
            if self.is_in_obstacle(x_rand):
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

            if self.is_in_obstacle(x_new):
                continue

            self.map.append(x_new)
            self.parent[tuple(x_new)] = tuple(x_near)  # Update parent dictionary

            # Add points to the marker for visualization
            p1 = Point()
            p1.x, p1.y, p1.z = x_near[0], x_near[1], 0.0
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
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05
        path_marker.color.a = 1.0
        path_marker.color.r = 1.0
        path_marker.color.g = 1.0
        path_marker.color.b = 1.0

        current_point = goal_point
        while current_point is not None:
            point = Point()
            point.x, point.y, point.z = current_point[0], current_point[1], 0.0
            path_marker.points.insert(0, point)
            current_point = self.parent.get(tuple(current_point), None)  # Trace back using parent dictionary

        self.path_publisher_.publish(path_marker)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt()
        else:
            self.get_logger().info('Monitoring environment for changes...')
            # Add logic here to monitor environment and replan if necessary

        # Publish start and goal markers
        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''

'''
#final path
#2D Pose Estimate as a start point
#2D Goal Pose as a goal point
#Created Obstacle
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
import numpy as np
import random

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.publisher_ = self.create_publisher(Marker, 'rrt_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Marker, 'final_path', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = None  # Initialize as None
        self.goal = None  # Initialize as None
        self.D = 0.3   #good paraeter 0.3meter
        self.max_iter = 50 #good 50iteration
        self.map = []  # Start with an empty map
        self.parent = {}  # Initialize parent dictionary

        # Define the map size
        self.map_size = [-5, 5, -5, 5]  # [xmin, xmax, ymin, ymax]

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

        # Initialize marker for obstacles
        self.obstacle_marker = Marker()
        self.obstacle_marker.header.frame_id = "map"
        self.obstacle_marker.type = Marker.CUBE_LIST
        self.obstacle_marker.action = Marker.ADD
        self.obstacle_marker.scale.x = 1.0  # Set to size of the cube obstacles
        self.obstacle_marker.scale.y = 1.0
        self.obstacle_marker.scale.z = 1.0
        self.obstacle_marker.color.a = 1.0
        self.obstacle_marker.color.r = 1.0
        self.obstacle_marker.color.g = 0.0
        self.obstacle_marker.color.b = 0.0

        self.obstacles = self.create_obstacles()  # Add obstacles

        self.path_found = False
        self.iterations = 0  # Initialize the counter
        self.max_iterations = 100  # Set the desired number of iterations

    def create_obstacles(self):
        # Define some example obstacles as cubes with center and size (half-extent)
        obstacles = [
            {'center': np.array([2.0, 2.0]), 'half_extent': 0.5},
            {'center': np.array([-1.0, -2.0]), 'half_extent': 0.5},
            {'center': np.array([3.0, -1.0]), 'half_extent': 0.5}
        ]
        for obstacle in obstacles:
            p = Point()
            p.x = obstacle['center'][0]
            p.y = obstacle['center'][1]
            p.z = 0.0
            self.obstacle_marker.points.append(p)
        return obstacles

    def is_in_obstacle(self, point):
        for obstacle in self.obstacles:
            if (obstacle['center'][0] - obstacle['half_extent'] <= point[0] <= obstacle['center'][0] + obstacle['half_extent'] and
                obstacle['center'][1] - obstacle['half_extent'] <= point[1] <= obstacle['center'][1] + obstacle['half_extent']):
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
            self.marker.points.clear()
            self.path_found = False    

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def run_rrt(self):
        if self.start is None or self.goal is None:
            self.get_logger().info('Waiting for start and goal positions...')
            return

        for j in range(self.max_iter):
            x_rand = np.array([
                random.uniform(self.map_size[0], self.map_size[1]),
                random.uniform(self.map_size[2], self.map_size[3])
            ])
            
            if self.is_in_obstacle(x_rand):
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

            if self.is_in_obstacle(x_new):
                continue

            self.map.append(x_new)
            self.parent[tuple(x_new)] = tuple(x_near)  # Update parent dictionary

            # Add points to the marker for visualization
            p1 = Point()
            p1.x, p1.y, p1.z = x_near[0], x_near[1], 0.0
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
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05
        path_marker.color.a = 1.0
        path_marker.color.r = 1.0
        path_marker.color.g = 1.0
        path_marker.color.b = 1.0

        current_point = goal_point
        while current_point is not None:
            point = Point()
            point.x, point.y, point.z = current_point[0], current_point[1], 0.0
            path_marker.points.insert(0, point)
            current_point = self.parent.get(tuple(current_point), None)  # Trace back using parent dictionary

        self.path_publisher_.publish(path_marker)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt()
        else:
            self.get_logger().info('Monitoring environment for changes...')
            # Add logic here to monitor environment and replan if necessary

        # Publish start and goal markers
        self.publish_start_goal_markers()
        self.start_goal_publisher_.publish(self.obstacle_marker)  # Publish obstacles

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


'''
#final path
#2D Pose Estimate as a start point
#2D Goal Pose as a goal point
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
import numpy as np
import random

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.publisher_ = self.create_publisher(Marker, 'rrt_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Marker, 'final_path', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = None #np.array([0.0, 0.0])
        self.goal = None #np.array([-4.0, 3.0])  # Example goal position
        self.D = 0.2
        self.max_iter = 100
        self.map = []#[self.start]
        self.parent = {} #{tuple(self.start): None}  # Added parent dictionary

        # Define the map size
        self.map_size = [-5, 5, -5, 5]  # [xmin, xmax, ymin, ymax]

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

        # Set the positions of the start and goal markers
        #self.start_marker.pose.position.x = self.start[0] #the first  index of start[0.0, 0.0]
        #self.start_marker.pose.position.y = self.start[1] #the second index of start[0.0, 0.0]
        #self.start_marker.pose.position.z = 0.0

        #.goal_marker.pose.position.x = self.goal[0]   #the first  index of goal[1.0, 1.0]
        #self.goal_marker.pose.position.y = self.goal[1]   #the second index of goal[1.0, 1.0]
        #self.goal_marker.pose.position.z = 0.0
        
        #publish the start and goal markers once
        #self.publish_start_goal_markers()
    
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
            self.marker.points.clear()
            self.path_found = False    

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def run_rrt(self):
        if self.start is None or self.goal is None:
            self.get_logger().info('Waiting for start and goal positions...')
            return

        for j in range(self.max_iter):
            x_rand = np.array([
                random.uniform(self.map_size[0], self.map_size[1]),
                random.uniform(self.map_size[2], self.map_size[3])
            ])
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

            self.map.append(x_new)
            self.parent[tuple(x_new)] = tuple(x_near)  # Update parent dictionary

            # Add points to the marker for visualization
            p1 = Point()
            p1.x, p1.y, p1.z = x_near[0], x_near[1], 0.0
            p2 = Point()
            p2.x, p2.y, p2.z = x_new[0], x_new[1], 0.0

            self.marker.points.append(p1)
            self.marker.points.append(p2)

            if np.linalg.norm(x_new - self.goal) < self.D:
                self.path_found = True
                self.get_logger().info('Path found to the goal.')
                self.publish_final_path(x_new)  # Call to publish final pat
                break

        self.publisher_.publish(self.marker)

    def publish_final_path(self, goal_point):
        # Trace back from the goal to the start to get the path
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05
        path_marker.color.a = 1.0
        path_marker.color.r = 1.0
        path_marker.color.g = 1.0
        path_marker.color.b = 1.0

        current_point = goal_point
        while current_point is not None:
            point = Point()
            point.x, point.y, point.z = current_point[0], current_point[1], 0.0
            path_marker.points.insert(0, point)
            current_point = self.parent.get(tuple(current_point), None)  # Trace back using parent dictionary

        self.path_publisher_.publish(path_marker)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt()
        else:
            self.get_logger().info('Monitoring environment for changes...')
            # Add logic here to monitor environment and replan if necessary

        # Publish start and goal markers
        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

'''
#final path
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import random

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.publisher_ = self.create_publisher(Marker, 'rrt_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Marker, 'final_path', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = np.array([0.0, 0.0])
        self.goal = np.array([-4.0, 3.0])  # Example goal position
        self.D = 0.2
        self.max_iter = 100
        self.map = [self.start]
        self.parent = {tuple(self.start): None}  # Added parent dictionary

        # Define the map size
        self.map_size = [-5, 5, -5, 5]  # [xmin, xmax, ymin, ymax]

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

        # Set the positions of the start and goal markers
        self.start_marker.pose.position.x = self.start[0] #the first  index of start[0.0, 0.0]
        self.start_marker.pose.position.y = self.start[1] #the second index of start[0.0, 0.0]
        self.start_marker.pose.position.z = 0.0

        self.goal_marker.pose.position.x = self.goal[0]   #the first  index of goal[1.0, 1.0]
        self.goal_marker.pose.position.y = self.goal[1]   #the second index of goal[1.0, 1.0]
        self.goal_marker.pose.position.z = 0.0
        
        #publish the start and goal markers once
        #self.publish_start_goal_markers()
    
    def publish_start_goal_markers(self):
        self.start_goal_publisher_.publish(self.start_marker)
        self.start_goal_publisher_.publish(self.goal_marker)

    def run_rrt(self):
        for j in range(self.max_iter):
            x_rand = np.array([
                random.uniform(self.map_size[0], self.map_size[1]),
                random.uniform(self.map_size[2], self.map_size[3])
            ])
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

            self.map.append(x_new)
            self.parent[tuple(x_new)] = tuple(x_near)  # Update parent dictionary

            # Add points to the marker for visualization
            p1 = Point()
            p1.x, p1.y, p1.z = x_near[0], x_near[1], 0.0
            p2 = Point()
            p2.x, p2.y, p2.z = x_new[0], x_new[1], 0.0

            self.marker.points.append(p1)
            self.marker.points.append(p2)

            if np.linalg.norm(x_new - self.goal) < self.D:
                self.path_found = True
                self.get_logger().info('Path found to the goal.')
                self.publish_final_path(x_new)  # Call to publish final pat
                break

        self.publisher_.publish(self.marker)

    def publish_final_path(self, goal_point):
        # Trace back from the goal to the start to get the path
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05
        path_marker.color.a = 1.0
        path_marker.color.r = 1.0
        path_marker.color.g = 1.0
        path_marker.color.b = 1.0

        current_point = goal_point
        while current_point is not None:
            point = Point()
            point.x, point.y, point.z = current_point[0], current_point[1], 0.0
            path_marker.points.insert(0, point)
            current_point = self.parent.get(tuple(current_point), None)  # Trace back using parent dictionary

        self.path_publisher_.publish(path_marker)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt()
        else:
            self.get_logger().info('Monitoring environment for changes...')
            # Add logic here to monitor environment and replan if necessary

        # Publish start and goal markers
        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

'''
#show the start and goal point
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import random

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.publisher_ = self.create_publisher(Marker, 'rrt_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = np.array([0.0, 0.0])
        self.goal = np.array([-4.0, 3.0])  # Example goal position
        self.D = 0.2
        self.max_iter = 100
        self.map = [self.start]

        # Define the map size
        self.map_size = [-5, 5, -5, 5]  # [xmin, xmax, ymin, ymax]

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

        # Set the positions of the start and goal markers
        self.start_marker.pose.position.x = self.start[0] #the first  index of start[0.0, 0.0]
        self.start_marker.pose.position.y = self.start[1] #the second index of start[0.0, 0.0]
        self.start_marker.pose.position.z = 0.0

        self.goal_marker.pose.position.x = self.goal[0]   #the first  index of goal[1.0, 1.0]
        self.goal_marker.pose.position.y = self.goal[1]   #the second index of goal[1.0, 1.0]
        self.goal_marker.pose.position.z = 0.0
        
        #publish the start and goal markers once
        #self.publish_start_goal_markers()

    def run_rrt(self):
        for j in range(self.max_iter):
            x_rand = np.array([
                random.uniform(self.map_size[0], self.map_size[1]),
                random.uniform(self.map_size[2], self.map_size[3])
            ])
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

            self.map.append(x_new)

            # Add points to the marker for visualization
            p1 = Point()
            p1.x, p1.y, p1.z = x_near[0], x_near[1], 0.0
            p2 = Point()
            p2.x, p2.y, p2.z = x_new[0], x_new[1], 0.0

            self.marker.points.append(p1)
            self.marker.points.append(p2)

            if np.linalg.norm(x_new - self.goal) < self.D:
                self.path_found = True
                self.get_logger().info('Path found to the goal.')
                break

        self.publisher_.publish(self.marker)

    def publish_start_goal_markers(self):
        self.start_goal_publisher_.publish(self.start_marker)
        self.start_goal_publisher_.publish(self.goal_marker)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt()
        else:
            self.get_logger().info('Monitoring environment for changes...')
            # Add logic here to monitor environment and replan if necessary

        # Publish start and goal markers
        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

'''
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import random

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.publisher_ = self.create_publisher(Marker, 'rrt_path', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = np.array([0.0, 0.0])
        self.goal = np.array([1.0, 1.0])  # Example goal position
        self.D = 0.2
        self.max_iter = 1000
        self.map = [self.start]

        # Define the map size
        self.map_size = [-5, 5, -5, 5]  # [xmin, xmax, ymin, ymax]

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

        self.path_found = False
        self.iterations = 0  # Initialize the counter
        self.max_iterations = 100  # Set the desired number of iterations

    def run_rrt(self):
        for j in range(self.max_iter):
            x_rand = np.array([
                random.uniform(self.map_size[0], self.map_size[1]),
                random.uniform(self.map_size[2], self.map_size[3])
            ])
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

            self.map.append(x_new)

            # Add points to the marker for visualization
            p1 = Point()
            p1.x, p1.y, p1.z = x_near[0], x_near[1], 0.0
            p2 = Point()
            p2.x, p2.y, p2.z = x_new[0], x_new[1], 0.0

            self.marker.points.append(p1)
            self.marker.points.append(p2)

            if np.linalg.norm(x_new - self.goal) < self.D:
                self.path_found = True
                self.get_logger().info('Path found to the goal.')
                break

        self.publisher_.publish(self.marker)

    def timer_callback(self):
        if not self.path_found:
            self.run_rrt()
        else:
            self.get_logger().info('Monitoring environment for changes...')
            # Add logic here to monitor environment and replan if necessary

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

'''
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import random

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.publisher_ = self.create_publisher(Marker, 'rrt_path', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = np.array([0.0, 0.0]) #start point (x,y), meter
        self.D = 0.2 #0.2m
        self.iter = 0
        self.max_iter = 500 #the disired num of iteration
        self.map = [self.start]

        # Define the map size
        self.map_size = [-5, 5, -5, 5]  # [xmin, xmax, ymin, ymax]

        # Initialize marker for visualization
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.LINE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.03 #tree leave size
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

    def timer_callback(self):
        for j in range(self.max_iter):
        #if self.iter < self.max_iter:
            #x_rand = np.array([15.0 * random.random(), 15.0 * random.random()])
            x_rand = np.array([
                random.uniform(self.map_size[0], self.map_size[1]),
                random.uniform(self.map_size[2], self.map_size[3])
            ])
            d_min = float('inf')
            #d_min = float(10)
            i_min = 0
            
            for i, point in enumerate(self.map):
                d = np.linalg.norm(point - x_rand)
                if d < d_min:
                    d_min = d
                    i_min = i

            x_near = self.map[i_min]
            v = x_rand - x_near
            x_new = x_near + (v / np.linalg.norm(v)) * self.D

            self.map.append(x_new)

            # Add points to the marker for visualization
            p1 = Point()
            p1.x, p1.y, p1.z = x_near[0], x_near[1], 0.0
            p2 = Point()
            p2.x, p2.y, p2.z = x_new[0], x_new[1], 0.0

            self.marker.points.append(p1)
            self.marker.points.append(p2)

        self.publisher_.publish(self.marker)

            #self.iter +=1

        #else:
        #    self.get_logger().info('Reached the maximum number of iterations, stopping...')
        #    self.timer.cancel()  # Stop the timer and end the algorithm

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
