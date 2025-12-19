import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import csv

class PathLogger(Node):
    def __init__(self):
        super().__init__('path_logger')
        self.subscription = self.create_subscription(Path, '/planned_path', self.listener_callback, 10)
        self.csv_file = open('planned_path.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x', 'y'])
        self.previous_coords = []

    def listener_callback(self, msg):
        current_coords = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

        # Check if the current coordinates are different from the previous ones
        if current_coords != self.previous_coords:
            timestamp = self.get_clock().now().to_msg().sec
            for x, y in current_coords:
                self.csv_writer.writerow([timestamp, x, y])
            self.previous_coords = current_coords

    def __del__(self):
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    path_logger = PathLogger()
    rclpy.spin(path_logger)
    path_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
