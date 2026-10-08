#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
import sys

class RPiTelemetryTest(Node):
    def __init__(self):
        super().__init__('test_rpi_telemetry')
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, qos_profile_sensor_data)
        
        self.has_heartbeat = False
        self.has_imu = False
        
        self.get_logger().info("=========================================")
        self.get_logger().info("PHYSICAL TELEMETRY TEST STARTED")
        self.get_logger().info("Waiting for Pixhawk MAVLink Heartbeat on /dev/ttyAMA0...")
        self.get_logger().info("=========================================")

    def state_callback(self, msg):
        if msg.connected and not self.has_heartbeat:
            self.has_heartbeat = True
            self.get_logger().info(f"[PASS] MAVLink Heartbeat acquired! Mode: {msg.mode}")
            self.check_pass()

    def imu_callback(self, msg):
        if not self.has_imu:
            self.has_imu = True
            self.get_logger().info("[PASS] IMU High-Frequency stream acquired!")
            self.check_pass()

    def check_pass(self):
        if self.has_heartbeat and self.has_imu:
            self.get_logger().info("\n\n>>> SUCCESS! Pixhawk UART Telemetry is fully operational! <<<\n")
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = RPiTelemetryTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
