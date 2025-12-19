#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can
from custom_messages.msg import MotorFeedback, MotorCommand
import struct
import subprocess
import time

def run_subprocess(cmd):
    """Executes a command in the subprocess and returns the result."""
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

class MotorNode(Node, can.Listener):
    def __init__(self):
        Node.__init__(self, 'motor_node')  # Changed node name to MotorNode
        self.setup_can_interface()
        self.encoder_publisher = self.create_publisher(MotorFeedback, '/motor_feedback', 10)
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
        self.notifier = can.Notifier(self.bus, [self])
        self.motor_command_subscriber = self.create_subscription(MotorCommand, '/publish_motor', self.callback, 1000)
        self.canMsgData = [0, 0, 0, 0, 0, 0, 0, 0]

    def callback(self, msg):
        self.canMsgData[0] = (
            msg.speedmode +
            (msg.stop << 1) +
            (msg.reset << 2) +
            (msg.voltagemode << 3)
        )
        goal_bytes = struct.pack("<f", msg.goal)
        self.canMsgData[2:6] = goal_bytes
        
        can_msg = can.Message(arbitration_id=msg.motor_id, data=self.canMsgData, is_extended_id=False)
        try:
            self.bus.send(can_msg)
        except can.CanError:
            pass

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

    def on_message_received(self, msg):
        if 128 < msg.arbitration_id < 256:
            feedback_msg = MotorFeedback()
            feedback_msg.motor_id = msg.arbitration_id
            feedback_msg.position = struct.unpack_from("<f", msg.data, 0)[0]
            feedback_msg.speed = struct.unpack_from("<f", msg.data, 4)[0]
            self.encoder_publisher.publish(feedback_msg)

    def shutdown(self):
        """Shuts down the CAN interface and other resources gracefully."""
        self.notifier.stop()
        self.bus.shutdown()
        self.destroy_node()
        # Shutdown the can0 interface
        run_subprocess(["sudo", "ip", "link", "set", "can0", "down"])
        self.get_logger().info("Shutdown CAN interface.")

def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorNode()
    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
