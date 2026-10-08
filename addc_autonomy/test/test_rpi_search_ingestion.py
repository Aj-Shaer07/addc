#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import WaypointList
from rclpy.qos import qos_profile_sensor_data
import sys

class RPiSearchIngestionTest(Node):
    def __init__(self):
        super().__init__('test_rpi_search')
        self.sub = self.create_subscription(WaypointList, '/mavros/mission/waypoints', self.callback, qos_profile_sensor_data)
        self.get_logger().info("=========================================")
        self.get_logger().info("GROUND-TO-AIR TELEMETRY TEST STARTED")
        self.get_logger().info("Please run 'python3 search_area_setup.py' on your laptop and click Upload...")
        self.get_logger().info("=========================================")

    def callback(self, msg):
        wps = [wp for wp in msg.waypoints if wp.command == 16]
        if len(wps) > 1:
            self.get_logger().info(f"\n\n>>> SUCCESS! Received {len(wps)} GPS grid waypoints from Laptop GCS! <<<\n")
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = RPiSearchIngestionTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
