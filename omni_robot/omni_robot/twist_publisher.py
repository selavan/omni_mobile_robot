import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPublisher(Node):
  def __init__(self):
    super().__init__('twist_publisher')
    self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
    self.timer_ = self.create_timer(0.01, self.publish_twist)
    self.get_logger().info('Twist publisher node has been initialized.')

  def publish_twist(self):
    twist_msg = Twist()
    # Set the linear and angular velocity values
    twist_msg.linear.x = 0.0  # 0.015m/s - 2.0m/s (forward)
    twist_msg.linear.y = 0.0  # 0.015m/s - 2.0m/s (right)
    twist_msg.linear.z = 0.0  # 0.015m/s - 2.0m/s (upward) - be cautious with upward movement
    twist_msg.angular.z = 0.0  # Example angular velocity value (no rotation)

    self.publisher_.publish(twist_msg)
    self.get_logger().info('Linear Velocity (x, y, z): {:.2f}, {:.2f}, {:.2f}, Angular Velocity (z): {:.2f}'.format(
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z, twist_msg.angular.z))

def main(args=None):
  rclpy.init(args=args)
  twist_publisher = TwistPublisher()
  rclpy.spin(twist_publisher)
  twist_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()



'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(0.01, self.publish_twist)
        self.get_logger().info('Twist publisher node has been initialized.')

    def publish_twist(self):
        twist_msg = Twist()
        # Set the linear and angular velocity values
        twist_msg.linear.x = 0.5  # 0.015m/s - 2.0m/s
        twist_msg.linear.y = 1.0  # 0.015m/s - 2.0m/s
        twist_msg.angular.z = 0.0  # Example angular velocity value

        self.publisher_.publish(twist_msg)
        #self.get_logger().info('Twist message published.')
        #self.get_logger().info('Twist msg pub: {}'.format(twist_msg))
        self.get_logger().info('Linear Velocity (x): {:.2f}, Angular Velocity (z): {:.2f}'.format(twist_msg.linear.x, twist_msg.angular.z))



def main(args=None):
    rclpy.init(args=args)
    twist_publisher = TwistPublisher()
    rclpy.spin(twist_publisher)
    twist_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
'''
