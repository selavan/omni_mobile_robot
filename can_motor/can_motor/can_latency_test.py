#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can
from custom_messages.msg import MotorFeedback
import struct
import subprocess
import time

def run_subprocess(cmd):
    """Executes a command in the subprocess and returns the result."""
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

class MotorFeedbackNode(Node, can.Listener):
    def __init__(self):
        Node.__init__(self, 'motor_feedback_node')
        self.setup_can_interface()
        self.publisher = self.create_publisher(MotorFeedback, '/motor_feedback', 10)
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
        self.notifier = can.Notifier(self.bus, [self])
        self.last_send_time = None
        self.create_timer(2, self.send_message)

    def setup_can_interface(self):
        """Checks and sets up the CAN interface."""
        result = run_subprocess(["ip", "link", "show", "can0"])
        if b"state UP" in result.stdout:
            self.get_logger().info("CAN interface is already up")
        else:
            result = run_subprocess(["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "1000000"])
            if result.returncode == 0:
                self.get_logger().info("CAN interface is up")
            else:
                self.get_logger().info("CAN failed to setup")

    def send_message(self):
        message = can.Message(arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7], is_extended_id=False)
        self.bus.send(message)
        self.last_send_time = time.time()
        self.get_logger().info('Sent CAN message.')

    def on_message_received(self, msg):
        if self.last_send_time:
            round_trip_time = time.time() - self.last_send_time
            self.get_logger().info(f'Received response in {round_trip_time} seconds')
            feedback_msg = MotorFeedback()
            feedback_msg.motor_id = msg.arbitration_id
            feedback_msg.position, feedback_msg.speed = struct.unpack_from("<ff", msg.data)
            self.publisher.publish(feedback_msg)
            self.last_send_time = None  # Reset the timer for the next message

    def shutdown(self):
        """Shuts down the CAN interface and other resources gracefully."""
        self.notifier.stop()
        self.bus.shutdown()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_feedback_node = MotorFeedbackNode()
    try:
        rclpy.spin(motor_feedback_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_feedback_node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
