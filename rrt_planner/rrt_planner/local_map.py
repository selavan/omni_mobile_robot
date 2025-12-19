import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np

class LocalMapPublisher(Node):
    def __init__(self):
        super().__init__('local_map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'local_map', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.obstacle_1_pos = (-2.5, 2.5)
        self.obstacle_2_pos = (2.5, 0.5)
        self.obstacle_size = 1.0

    def timer_callback(self):
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"

        map_msg.info = MapMetaData()
        map_msg.info.map_load_time = self.get_clock().now().to_msg()
        map_msg.info.resolution = 0.1
        map_msg.info.width = 100
        map_msg.info.height = 100
        map_msg.info.origin.position.x = -5.0
        map_msg.info.origin.position.y = -5.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        map_data = np.full((100, 100), -1)
        map_data[0, :] = 100
        map_data[:, 0] = 100
        map_data[-1, :] = 100
        map_data[:, -1] = 100

        self.add_obstacle(map_data, self.obstacle_1_pos, self.obstacle_size)
        self.add_obstacle(map_data, self.obstacle_2_pos, self.obstacle_size)

        for i in range(10, 90):
            for j in range(10, 90):
                if map_data[i, j] != 100:
                    map_data[i, j] = 0

        map_msg.data = map_data.flatten().tolist()
        self.publisher_.publish(map_msg)
        self.get_logger().info('Publishing local map with dynamic obstacles')

    def add_obstacle(self, map_data, position, size):
        center_x, center_y = position
        size_in_cells = int(size / 0.1)
        half_size = size_in_cells // 2

        center_x_idx = int((center_x + 5.0) / 0.1)
        center_y_idx = int((center_y + 5.0) / 0.1)

        for i in range(center_x_idx - half_size, center_x_idx + half_size + 1):
            for j in range(center_y_idx - half_size, center_y_idx + half_size + 1):
                if 0 <= i < 100 and 0 <= j < 100:
                    map_data[i, j] = 100

def main(args=None):
    rclpy.init(args=args)
    local_map_publisher = LocalMapPublisher()
    rclpy.spin(local_map_publisher)
    local_map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np

class LocalMapPublisher(Node):
    def __init__(self):
        super().__init__('local_map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'local_map', 10)
        
        # Set timer periods
        self.timer_period = 0.1  # seconds (publishing interval)
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Define the position and size of the unknown obstacles (in meters)
        self.obstacle_1_pos = (-2.5, 2.5)  # Center of obstacle 1
        self.obstacle_2_pos = (2.5, 0.5)   # Center of obstacle 2
        self.obstacle_size = 1.0  # 1x1 meter

    def timer_callback(self):
        map_msg = self.create_obstacle_map_msg()
        self.publisher_.publish(map_msg)
        self.get_logger().info('Publishing local map with dynamic obstacles')

    def create_obstacle_map_msg(self):
        map_msg = OccupancyGrid()

        # Set the header
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"

        # Set the info
        map_msg.info = MapMetaData()
        map_msg.info.map_load_time = self.get_clock().now().to_msg()
        map_msg.info.resolution = 0.1  # Each cell is 10cm x 10cm
        map_msg.info.width = 100  # 10 meters / 0.1 meter per cell
        map_msg.info.height = 100  # 10 meters / 0.1 meter per cell
        map_msg.info.origin.position.x = -5.0  # Center the map
        map_msg.info.origin.position.y = -5.0  # Center the map
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Create the map data (100x100 grid)
        map_data = np.full((100, 100), -1)  # Unknown space

        # Add the unknown obstacles
        self.add_obstacle(map_data, self.obstacle_1_pos, self.obstacle_size)
        self.add_obstacle(map_data, self.obstacle_2_pos, self.obstacle_size)

        # Flatten the 2D array to 1D and set it as the data of the OccupancyGrid message
        map_msg.data = map_data.flatten().tolist()

        return map_msg

    def add_obstacle(self, map_data, position, size):
        # Calculate the indices in the map corresponding to the obstacle
        center_x, center_y = position
        size_in_cells = int(size / 0.1)
        half_size = size_in_cells // 2

        center_x_idx = int((center_x + 5.0) / 0.1)
        center_y_idx = int((center_y + 5.0) / 0.1)

        for i in range(center_x_idx - half_size, center_x_idx + half_size + 1):
            for j in range(center_y_idx - half_size, center_y_idx + half_size + 1):
                if 0 <= i < 100 and 0 <= j < 100:
                    map_data[i, j] = 100  # Unknown space

def main(args=None):
    rclpy.init(args=args)
    local_map_publisher = LocalMapPublisher()
    rclpy.spin(local_map_publisher)
    local_map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''