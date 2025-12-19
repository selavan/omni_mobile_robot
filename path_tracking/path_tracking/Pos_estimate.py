#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class TFStateEstimator(Node):
    def __init__(self):
        super().__init__('tf_state_estimator')
        
        # Create a TransformListener and Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher for Odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom_estimate', 10)
        
        # Timer to periodically fetch and publish the transformation
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        try:
            now = rclpy.time.Time(seconds=0)  # Use time=0 for the latest transform
            # Lookup the transformation from 'map' to 'odom'
            transform = self.tf_buffer.lookup_transform('map', 'odom', now)
            self.publish_odometry(transform)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform: {e}')

    def publish_odometry(self, transform: TransformStamped):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'odom'

        # Set the position
        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.position.z = transform.transform.translation.z
        
        # Set the orientation
        odom.pose.pose.orientation = transform.transform.rotation
        
        # Set the velocity (optional, here just set to zero)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        # Publish the odometry message
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = TFStateEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
