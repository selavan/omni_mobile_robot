#'''
#calculate distance on path executed 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import atexit
import math
import time

class ExecutedPathPublisher(Node):
    def __init__(self):
        super().__init__('executed_path_publisher')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.executed_path_pub = self.create_publisher(Path, '/executed_path', 10)
        self.executed_path = Path()
        self.executed_path.header.frame_id = 'map'
        self.current_pose = None
        self.start_time = None
        self.total_distance = 0.0
        self.goal_position = None
        self.goal_tolerance = 0.1  # Tolerance for reaching the goal position

        # Register shutdown callback
        atexit.register(self.stop_robot)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

        # Append the current pose to the executed path
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = self.current_pose
        self.executed_path.poses.append(pose)

        # Calculate the distance traveled
        if len(self.executed_path.poses) > 1:
            previous_pose = self.executed_path.poses[-2].pose
            distance = self.calculate_distance(previous_pose, self.current_pose)
            self.total_distance += distance

        # Set start time if not already set
        if self.start_time is None:
            self.start_time = time.time()

        # Check if the robot has reached the goal
        if self.goal_position and self.has_reached_goal(self.current_pose, self.goal_position):
            self.stop_robot()

        # Publish the executed path periodically
        self.executed_path_pub.publish(self.executed_path)

    def stop_robot(self):
        if self.start_time and self.goal_position:
            total_time = time.time() - self.start_time
            self.get_logger().info(f'Total distance traveled: {self.total_distance:.2f} meters')
            self.get_logger().info(f'Total time traveled: {total_time:.2f} seconds')

    def destroy_node(self):
        # Ensure the robot stops before the node is destroyed
        self.stop_robot()
        super().destroy_node()

    def calculate_distance(self, pose1, pose2):
        x1, y1 = pose1.position.x, pose1.position.y
        x2, y2 = pose2.position.x, pose2.position.y
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def has_reached_goal(self, current_pose, goal_position):
        distance_to_goal = self.calculate_distance(current_pose, goal_position)
        return distance_to_goal <= self.goal_tolerance

    def set_goal_position(self, x, y):
        self.goal_position = PoseStamped().pose
        self.goal_position.position.x = x
        self.goal_position.position.y = y

def main(args=None):
    rclpy.init(args=args)
    executed_path_publisher = ExecutedPathPublisher()

    # Set the goal position (example coordinates)
    executed_path_publisher.set_goal_position(5.0, 5.0)  # Replace with actual goal coordinates

    rclpy.spin(executed_path_publisher)
    executed_path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#'''



'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import atexit

class ExecutedPathPublisher(Node):
    def __init__(self):
        super().__init__('executed_path_publisher')
        #self.odom_sub = self.create_subscription(Odometry, 'odom_estimate', self.odom_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.executed_path_pub = self.create_publisher(Path, '/executed_path', 10)
        self.executed_path = Path()
        self.executed_path.header.frame_id = 'map'
        self.current_pose = None

        # Register shutdown callback
        atexit.register(self.stop_robot)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

        # Append the current pose to the executed path
        pose = PoseStamped()
        pose.pose = self.current_pose
        self.executed_path.poses.append(pose)

        # Publish the executed path periodically
        self.executed_path_pub.publish(self.executed_path)

    def stop_robot(self):
        # Ensure the robot stops when the node is destroyed
        pass

    def destroy_node(self):
        # Ensure the robot stops before the node is destroyed
        self.stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    executed_path_publisher = ExecutedPathPublisher()
    rclpy.spin(executed_path_publisher)
    executed_path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''