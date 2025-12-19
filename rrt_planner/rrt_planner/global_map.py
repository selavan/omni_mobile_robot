import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np

class GlobalMapPublisher(Node):
    def __init__(self):
        super().__init__('global_map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'global_map', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.publish_map()

    def publish_map(self):
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

        for i in range(30, 70):
            map_data[i, 40] = 100
        for i in range(40, 60):
            map_data[50, i] = 100
        for i in range(20, 30):
            for j in range(20, 30):
                map_data[i, j] = 100
        for i in range(70, 80):
            for j in range(70, 80):
                map_data[i, j] = 100

        for i in range(10, 90):
            for j in range(10, 90):
                if map_data[i, j] != 100:
                    map_data[i, j] = 0
        
        map_msg.data = map_data.flatten().tolist()
        self.publisher_.publish(map_msg)
        self.get_logger().info('Publishing global map')

    def timer_callback(self):
        self.publish_map()

def main(args=None):
    rclpy.init(args=args)
    global_map_publisher = GlobalMapPublisher()
    rclpy.spin(global_map_publisher)
    global_map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np

class GlobalMapPublisher(Node):
    def __init__(self):
        super().__init__('global_map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'global_map', 10)
        self.timer_period = 0.1  # seconds (publishing interval)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        map_msg = self.create_map_msg()
        self.publisher_.publish(map_msg)
        self.get_logger().info('Publishing global map')

    def create_map_msg(self):
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

        # Define some obstacles
        # Creating a boundary around the edges
        map_data[0, :] = 100  # Top boundary
        map_data[:, 0] = 100  # Left boundary
        map_data[-1, :] = 100 # Bottom boundary
        map_data[:, -1] = 100 # Right boundary

        # Add some complex obstacles inside the map
        for i in range(30, 70):
            map_data[i, 40] = 100  # Vertical wall

        for i in range(40, 60):
            map_data[50, i] = 100  # Horizontal wall

        for i in range(20, 30):
            for j in range(20, 30):
                map_data[i, j] = 100  # Square obstacle

        for i in range(70, 80):
            for j in range(70, 80):
                map_data[i, j] = 100  # Another square obstacle

        # Set some free space explicitly
        for i in range(10, 90):
            for j in range(10, 90):
                if map_data[i, j] != 100:
                    map_data[i, j] = 0  # Free space

        # Flatten the 2D array to 1D and set it as the data of the OccupancyGrid message
        map_msg.data = map_data.flatten().tolist()

        return map_msg

def main(args=None):
    rclpy.init(args=args)
    global_map_publisher = GlobalMapPublisher()
    rclpy.spin(global_map_publisher)
    global_map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''