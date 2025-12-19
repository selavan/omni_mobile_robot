import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import heapq

class AStarNode(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        self.publisher_ = self.create_publisher(Marker, 'a_star_path', 10)
        self.start_goal_publisher_ = self.create_publisher(Marker, 'start_goal', 10)
        self.path_publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalpose_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start = None
        self.goal = None
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None

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

    def initialpose_callback(self, msg):
        self.start = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.start_marker.pose.position.x = self.start[0]
        self.start_marker.pose.position.y = self.start[1]
        self.start_marker.pose.position.z = 0.0
        self.start_goal_publisher_.publish(self.start_marker)
        self.get_logger().info(f'Start position set to: {self.start}')
        self.reset_a_star()

    def goalpose_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.goal_marker.pose.position.x = self.goal[0]
        self.goal_marker.pose.position.y = self.goal[1]
        self.goal_marker.pose.position.z = 0.0
        self.start_goal_publisher_.publish(self.goal_marker)
        self.get_logger().info(f'Goal position set to: {self.goal}')
        self.reset_a_star()

    def reset_a_star(self):
        if self.start is not None and self.goal is not None:
            self.marker.points.clear()
            self.path_found = False

    def publish_start_goal_markers(self):
        if self.start is not None:
            self.start_goal_publisher_.publish(self.start_marker)
        if self.goal is not None:
            self.start_goal_publisher_.publish(self.goal_marker)

    def a_star_planning(self):
        if self.start is None or self.goal is None:
            self.get_logger().info('Waiting for start and goal positions...')
            return
        if self.map_data is None:
            self.get_logger().info('Waiting for map data...')
            return

        start = tuple(self.start)
        goal = tuple(self.goal)
        open_list = []
        closed_list = set()
        parent = {}
        g_cost = {start: 0}
        f_cost = {start: self.heuristic(start, goal)}
        heapq.heappush(open_list, (f_cost[start], start))

        while open_list:
            _, current = heapq.heappop(open_list)
            closed_list.add(current)

            if current == goal:
                self.path_found = True
                self.get_logger().info('Path found to the goal.')
                self.publish_final_path(current, parent)
                return

            for neighbor in self.get_neighbors(current):
                if neighbor in closed_list:
                    continue

                tentative_g_cost = g_cost[current] + self.distance(current, neighbor)
                if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]:
                    parent[neighbor] = current
                    g_cost[neighbor] = tentative_g_cost
                    f_cost[neighbor] = tentative_g_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_cost[neighbor], neighbor))

                    p1 = Point()
                    p1.x, p1.y, p1.z = current[0], current[1], 0.0
                    p2 = Point()
                    p2.x, p2.y, p2.z = neighbor[0], neighbor[1], 0.0

                    self.marker.points.append(p1)
                    self.marker.points.append(p2)

        self.publisher_.publish(self.marker)

    def get_neighbors(self, point):
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        neighbors = []
        for direction in directions:
            neighbor = (point[0] + direction[0] * self.map_resolution, point[1] + direction[1] * self.map_resolution)
            if not self.is_in_obstacle(neighbor):
                neighbors.append(neighbor)
        return neighbors

    def heuristic(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def publish_final_path(self, goal_point, parent):
        planned_path = Path()
        planned_path.header.frame_id = "map"

        current_point = goal_point
        while current_point in parent:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = current_point[0]
            pose_stamped.pose.position.y = current_point[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            planned_path.poses.insert(0, pose_stamped)

            current_point = parent[current_point]

        self.path_publisher_.publish(planned_path)

    def timer_callback(self):
        if not self.path_found:
            self.a_star_planning()
        else:
            self.get_logger().info('Monitoring environment for changes...')
        self.publish_start_goal_markers()

def main(args=None):
    rclpy.init(args=args)
    a_star_node = AStarNode()
    rclpy.spin(a_star_node)
    a_star_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
