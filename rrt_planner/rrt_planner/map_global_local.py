#'''
#map 10*10meter
#with complex known obstacle
#update to global and local map
#the local map will publish after 5s
#remove map_ms
#both static and dynamic on the same /map topic
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        
        # Set timer periods
        self.timer_period = 0.1  # seconds (publishing interval)
        self.condition_time = 10.0  # seconds (time before dynamic obstacles appear)
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()

        # Define the position and size of the unknown obstacles (in meters)
        self.obstacle_1_pos = (-2.5, 2.5)  # Center of obstacle 1
        self.obstacle_2_pos = (2.5, 0.5)   # Center of obstacle 2
        self.obstacle_size = 1.0  # 1x1 meter

    def timer_callback(self):
        # Publish the map
        map_msg = self.create_map_msg()
        current_time = self.get_clock().now()
        if (current_time - self.start_time).nanoseconds / 1e9 >= self.condition_time:
            self.add_obstacle(map_msg)
        self.publisher_.publish(map_msg)
        self.get_logger().info('Publishing map with possible dynamic obstacles')

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

    def add_obstacle(self, map_msg):
        map_data = np.array(map_msg.data).reshape((100, 100))
        self.add_single_obstacle(map_data, self.obstacle_1_pos, self.obstacle_size)
        self.add_single_obstacle(map_data, self.obstacle_2_pos, self.obstacle_size)
        map_msg.data = map_data.flatten().tolist()

    def add_single_obstacle(self, map_data, position, size):
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
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#'''

'''
#map 10*10meter
#with complex known obstacle
#update to global and local map
#the local map will publish after 5s
#remove map_ms
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.global_map_publisher = self.create_publisher(OccupancyGrid, 'global_map', 10)
        self.obstacle_map_publisher = self.create_publisher(OccupancyGrid, 'local_map', 10)
        
        # Set timer periods
        self.timer_period = 0.1  # seconds (publishing interval)
        self.condition_time = 10.0  # seconds (time before dynamic obstacles appear)
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()

        # Define the position and size of the unknown obstacles (in meters)
        self.obstacle_1_pos = (-2.5, 2.5)  # Center of obstacle 1
        self.obstacle_2_pos = (2.5, 0.5)   # Center of obstacle 2
        self.obstacle_size = 1.0  # 1x1 meter

    def timer_callback(self):
        # Publish the global map
        global_map_msg = self.create_map_msg()
        self.global_map_publisher.publish(global_map_msg)
        self.get_logger().info('Publishing global map')

        # Check if the condition time has passed
        current_time = self.get_clock().now()
        if (current_time - self.start_time).nanoseconds / 1e9 >= self.condition_time:
            obstacle_map_msg = self.create_obstacle_map_msg(global_map_msg)
            self.obstacle_map_publisher.publish(obstacle_map_msg)
            self.get_logger().info('Publishing local map with dynamic obstacles')

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

    def create_obstacle_map_msg(self, base_map_msg):
        obstacle_map_msg = OccupancyGrid()
        obstacle_map_msg.header = base_map_msg.header
        obstacle_map_msg.info = base_map_msg.info

        # Create a copy of the base map data and add the unknown obstacles
        map_data = np.array(base_map_msg.data).reshape((100, 100))
        self.add_obstacle(map_data, self.obstacle_1_pos, self.obstacle_size)
        self.add_obstacle(map_data, self.obstacle_2_pos, self.obstacle_size)

        # Flatten the 2D array to 1D and set it as the data of the OccupancyGrid message
        obstacle_map_msg.data = map_data.flatten().tolist()

        return obstacle_map_msg

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
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

'''
#map 10*10meter
#with complex known obstacle
#update to global and local map
#the local map will publish after 5s
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.global_map_publisher = self.create_publisher(OccupancyGrid, 'global_map', 10)
        self.obstacle_map_publisher = self.create_publisher(OccupancyGrid, 'local_map', 10)
        
        # Set timer periods
        self.timer_period = 0.1  # seconds (publishing interval)
        self.condition_time = 5.0  # seconds (time before dynamic obstacles appear)
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()

        # Define the position and size of the unknown obstacles (in meters)
        self.obstacle_1_pos = (-2.5, 2.5)  # Center of obstacle 1
        self.obstacle_2_pos = (2.5, 0.5)   # Center of obstacle 2
        self.obstacle_size = 1.0  # 1x1 meter

    def timer_callback(self):
        # Publish the global map
        global_map_msg = self.create_map_msg()
        self.global_map_publisher.publish(global_map_msg)
        self.get_logger().info('Publishing global map')

        # Check if the condition time has passed
        current_time = self.get_clock().now()
        if (current_time - self.start_time).nanoseconds / 1e9 >= self.condition_time:
            obstacle_map_msg = self.create_obstacle_map_msg()
            self.obstacle_map_publisher.publish(obstacle_map_msg)
            self.get_logger().info('Publishing local map with dynamic obstacles')

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
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''

'''
#map 10*10meter
#with complex known obstacle
#update to global and local map
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.global_map_publisher = self.create_publisher(OccupancyGrid, 'global_map', 10)
        self.obstacle_map_publisher = self.create_publisher(OccupancyGrid, 'local_map', 10)
        
        # Set timer periods
        self.timer_period = 0.1  # seconds (publishing interval)
        self.toggle_timer_period = 3.0  # seconds (obstacle toggle interval)
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.toggle_timer = self.create_timer(self.toggle_timer_period, self.toggle_obstacles)
        self.obstacles_visible = False
        self.condition_met = False
        self.counter = 0

        # Define the position and size of the unknown obstacles (in meters)
        self.obstacle_1_pos = (-2.5, 2.5)  # Center of obstacle 1
        self.obstacle_2_pos = (2.5, 0.5)   # Center of obstacle 2
        self.obstacle_size = 1.0  # 1x1 meter

    def timer_callback(self):
        # Publish the global map
        global_map_msg = self.create_map_msg()
        self.global_map_publisher.publish(global_map_msg)
        self.get_logger().info('Publishing global map')

        # Check condition (e.g., after 10 timer callbacks)
        if self.counter >= 10:
            self.condition_met = True

        # Publish the obstacle map if condition is met
        if self.condition_met:
            obstacle_map_msg = self.create_obstacle_map_msg()
            self.obstacle_map_publisher.publish(obstacle_map_msg)
            self.get_logger().info('Publishing obstacle map')

        # Increment the counter
        self.counter += 1

    def toggle_obstacles(self):
        # Toggle the visibility of the unknown obstacles if the condition is met
        if self.condition_met:
            self.obstacles_visible = not self.obstacles_visible

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

    def create_obstacle_map_msg(self):
        map_msg = self.create_map_msg()

        # Add or remove unknown obstacles based on the toggle state
        if self.obstacles_visible:
            map_data = np.array(map_msg.data).reshape((100, 100))
            self.add_obstacle(map_data, self.obstacle_1_pos, self.obstacle_size)
            self.add_obstacle(map_data, self.obstacle_2_pos, self.obstacle_size)
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
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
#map 10*10meter
#with complex known obstacle
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        
        # Set timer periods
        self.timer_period = 0.1  # seconds (publishing interval)
        self.toggle_timer_period = 1.0  # seconds (obstacle toggle interval)
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.toggle_timer = self.create_timer(self.toggle_timer_period, self.toggle_obstacles)
        self.obstacles_visible = False

        # Define the position and size of the unknown obstacles (in meters)
        self.obstacle_1_pos = (-2.5, 2.5)  # Center of obstacle 1
        self.obstacle_2_pos = (2.5, 0.5)    # Center of obstacle 2
        self.obstacle_size = 1.0  # 1x1 meter

    def timer_callback(self):
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

        # Add or remove unknown obstacles based on the toggle state
        if self.obstacles_visible:
            self.add_obstacle(map_data, self.obstacle_1_pos, self.obstacle_size)
            self.add_obstacle(map_data, self.obstacle_2_pos, self.obstacle_size)

        # Flatten the 2D array to 1D and set it as the data of the OccupancyGrid message
        map_msg.data = map_data.flatten().tolist()

        # Publish the map
        self.publisher_.publish(map_msg)
        self.get_logger().info('Publishing complex map with dynamic obstacles')

    def toggle_obstacles(self):
        # Toggle the visibility of the unknown obstacles
        self.obstacles_visible = not self.obstacles_visible

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
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

'''
#map 10*10meter
#with complex known obstacle
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
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

        # Publish the map
        self.publisher_.publish(map_msg)
        self.get_logger().info('Publishing complex map')

def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

'''
#map 10*10meter
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
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

        # Create the map data (100x100 grid with all cells free)
        map_data = np.full((100, 100), -1)  # Unknown space
        for i in range(20, 80):
            for j in range(20, 80):
                map_data[i, j] = 0  # Free space

        # Flatten the 2D array to 1D and set it as the data of the OccupancyGrid message
        map_msg.data = map_data.flatten().tolist()

        # Publish the map
        self.publisher_.publish(map_msg)
        self.get_logger().info('Publishing map')

def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''