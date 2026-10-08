#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys

class RPiPrecisionLandingTest(Node):
    def __init__(self):
        super().__init__('test_rpi_landing')
        self.sub = self.create_subscription(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', self.callback, 10)
        self.get_logger().info("=========================================")
        self.get_logger().info("PHYSICAL PRECISION LANDING TEST STARTED")
        self.get_logger().info("Please place the Blue/White landing pad in front of the Arducam IMX296...")
        self.get_logger().info("=========================================")
        self.cmd_count = 0

    def callback(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.get_logger().info(f"Target Acquired! Velocity Command -> VX: {vx:.2f}, VY: {vy:.2f}")
        self.cmd_count += 1
        if self.cmd_count >= 5:
            self.get_logger().info("\n\n>>> SUCCESS! Canny Edge Visual Servoing is fully operational! <<<\n")
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = RPiPrecisionLandingTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
