#'''
#circle waypoints
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

class CircleWaypointPublisher(Node):
    def __init__(self):
        super().__init__('circle_waypoint_publisher')
        self.publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.radius = 2.0  # Radius of the circle
        self.num_points = 50  # Number of points in the circle
        self.center_x = 0.0  # X coordinate of the circle's center
        self.center_y = 0.0  # Y coordinate of the circle's center

    def timer_callback(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        theta = np.linspace(0, 2 * np.pi, self.num_points)

        for angle in theta:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = self.center_x + self.radius * np.cos(angle)
            pose.pose.position.y = self.center_y + self.radius * np.sin(angle)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation
            path.poses.append(pose)

        self.publisher_.publish(path)
        self.get_logger().info(f'Published {len(path.poses)} waypoints.')

def main(args=None):
    rclpy.init(args=args)
    node = CircleWaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#'''

'''
#predefine path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        path = Path()
        path.header.frame_id = "map"

        waypoints = [
            [0.0, 0.0],
            [3.0, 0.0],
            [3.0, 3.0],
            [0.0, 3.0],
            [0.0, 0.0]
        ]

        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.publisher_.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''