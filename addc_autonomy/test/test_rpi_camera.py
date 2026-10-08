#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class RPiCameraTest(Node):
    def __init__(self):
        super().__init__('test_rpi_camera')
        self.sub = self.create_subscription(String, '/addc/vision/decoded_digits', self.callback, 10)
        self.get_logger().info("=========================================")
        self.get_logger().info("PHYSICAL CAMERA TEST STARTED")
        self.get_logger().info("Please hold a QR code in front of the Arducam IMX296...")
        self.get_logger().info("=========================================")

    def callback(self, msg):
        self.get_logger().info(f"\n\n>>> SUCCESS! QR Code Decoded: {msg.data} <<<\n")
        self.get_logger().info("Camera Pipeline is fully operational!")
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = RPiCameraTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
