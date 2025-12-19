# map_publisher/map_publisher_node.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
import yaml
import numpy as np
import cv2

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher_node')

        # Load the map from the YAML file       
        
        map_yaml_path = '/map/map_5.yaml'
        with open(map_yaml_path, 'r') as file:
            map_metadata = yaml.safe_load(file)
        
        # Extract information from the YAML file
        self.map_image_path = map_metadata['image']
        self.map_resolution = map_metadata['resolution']
        self.map_origin = map_metadata['origin']
        self.occupied_thresh = map_metadata['occupied_thresh']
        self.free_thresh = map_metadata['free_thresh']
        
        # Load the map image
        self.map_image = cv2.imread(self.map_image_path, cv2.IMREAD_GRAYSCALE)
        self.map_image = np.flipud(self.map_image)

        # Convert the image to occupancy data
        self.map_data = np.zeros(self.map_image.shape, dtype=np.int8)
        self.map_data[self.map_image >= self.occupied_thresh * 255] = 100
        self.map_data[self.map_image <= self.free_thresh * 255] = 0
        self.map_data[(self.map_image > self.free_thresh * 255) & (self.map_image < self.occupied_thresh * 255)] = -1

        # Initialize the OccupancyGrid message
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = 'map'
        self.occupancy_grid.info = MapMetaData()
        self.occupancy_grid.info.width = self.map_data.shape[1]
        self.occupancy_grid.info.height = self.map_data.shape[0]
        self.occupancy_grid.info.resolution = self.map_resolution
        self.occupancy_grid.info.origin.position.x = self.map_origin[0]
        self.occupancy_grid.info.origin.position.y = self.map_origin[1]
        self.occupancy_grid.info.origin.position.z = self.map_origin[2]
        self.occupancy_grid.info.origin.orientation.w = 1.0
        self.occupancy_grid.data = self.map_data.flatten().tolist()

        # Create a publisher for the OccupancyGrid
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        
        # Publish the map periodically
        self.timer = self.create_timer(1.0, self.publish_map)

    def publish_map(self):
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.occupancy_grid)

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
