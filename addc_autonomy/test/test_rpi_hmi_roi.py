#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys

class RPiHMIRoiTest(Node):
    def __init__(self):
        super().__init__('test_rpi_hmi')
        self.sub = self.create_subscription(Float32MultiArray, '/addc/hmi/priority_roi', self.callback, 10)
        self.get_logger().info("=========================================")
        self.get_logger().info("TAILSCALE / FLUTTER HMI TEST STARTED")
        self.get_logger().info("Please open the Runner Flutter App, connect to Tailscale, and tap a priority ROI...")
        self.get_logger().info("=========================================")

    def callback(self, msg):
        if len(msg.data) == 4:
            self.get_logger().info(f"\n\n>>> SUCCESS! Received ROI Preemption Request: {msg.data} over Tailscale! <<<\n")
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = RPiHMIRoiTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
