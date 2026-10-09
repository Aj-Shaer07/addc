#!/usr/bin/env python3
"""
mission_control_node.py - Master Orchestrator for ADDC Autonomy
Coordinates MAVROS flight states, search grid activation, target discovery, and precision landing.
"""

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data

# MAVROS imports
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, StreamRate

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control')
        
        # Parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.declare_parameter('search_altitude', 3.0)
        
        self.search_altitude = float(self.get_parameter('search_altitude').value)
        
        # Internal States
        self.current_state = "INIT"
        self.mavros_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.target_found = False
        self.last_service_time = self.get_clock().now()
        
        # Dynamic Home Location (recorded at takeoff)
        self.home_x = 0.0
        self.home_y = 0.0
        
        # Initialize target pose structure cleanly with valid orientation
        self.target_pose.header.frame_id = "map"
        self.target_pose.pose.position.z = self.search_altitude
        self.target_pose.pose.orientation.w = 1.0

        # MAVROS Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self._mavros_state_cb, qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._mavros_pose_cb, qos_profile_sensor_data)
        
        # MAVROS Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # ADDC Subnets & Publishers
        self.vision_sub = self.create_subscription(String, '/addc/vision/decoded_digits', self._vision_cb, 10)
        self.search_wp_sub = self.create_subscription(PoseStamped, '/addc/search/waypoint', self._search_wp_cb, 10)
        self.search_activate_pub = self.create_publisher(Bool, '/addc/search/activate', 10)
        
        # MAVROS Services
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')
        
        # ADDC Services
        self.release_cam_client = self.create_client(Trigger, '/addc/vision/release_camera')
        self.landing_start_client = self.create_client(Trigger, '/addc/landing/start')
        
        # Main State Machine Loop (10 Hz)
        self.timer = self.create_timer(0.1, self._state_machine_loop)
        
        # Fault-Tolerant Watchdog
        self.health_stamps = {
            'qr_ros': self.get_clock().now(),
            'search_node': self.get_clock().now()
        }
        self.health_subs = {
            'qr_ros': self.create_subscription(Bool, '/addc/health/qr_ros', lambda msg: self._health_cb(msg, 'qr_ros'), 10),
            'search_node': self.create_subscription(Bool, '/addc/health/search_node', lambda msg: self._health_cb(msg, 'search_node'), 10)
        }
        self.drone_halted = False
        self.watchdog_timer = self.create_timer(0.2, self._watchdog_loop)
        
        self.get_logger().info("[Orchestrator] Mission Control Initialized. Waiting for FCU...")

    def _mavros_state_cb(self, msg: State):
        self.mavros_state = msg

    def _mavros_pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _vision_cb(self, msg: String):
        if msg.data and not self.target_found:
            self.get_logger().info(f"[Orchestrator] Target Confirmed! Decoded: {msg.data}")
            self.target_found = True
            self.current_state = "TARGET_FOUND"

    def _search_wp_cb(self, msg: PoseStamped):
        if self.current_state == "SEARCHING":
            self.target_pose = msg
            self.target_pose.pose.orientation.w = 1.0

    def _set_mode(self, mode: str):
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            self.set_mode_client.call_async(req)

    def _arm(self):
        if self.arm_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.arm_client.call_async(req)

    def _takeoff(self, altitude):
        if self.takeoff_client.service_is_ready():
            req = CommandTOL.Request()
            req.altitude = altitude
            req.latitude = float('nan')
            req.longitude = float('nan')
            self.takeoff_client.call_async(req)

    def _call_trigger_service(self, client):
        if client.service_is_ready():
            req = Trigger.Request()
            client.call_async(req)

    def _publish_search_activation(self, active: bool):
        msg = Bool()
        msg.data = active
        self.search_activate_pub.publish(msg)

    def _health_cb(self, msg: Bool, node_name: str):
        if msg.data:
            self.health_stamps[node_name] = self.get_clock().now()

    def _watchdog_loop(self):
        # Don't halt if not actively flying a mission
        if self.current_state not in ["SEARCHING", "TARGET_FOUND"]:
            return
            
        now = self.get_clock().now()
        crashed_nodes = []
        for node, stamp in self.health_stamps.items():
            if (now - stamp).nanoseconds / 1e9 > 2.0:
                crashed_nodes.append(node)
                
        if crashed_nodes and not self.drone_halted:
            self.get_logger().error(f"[Orchestrator] WATCHDOG TRIGGERED! {crashed_nodes} crashed! Halting drone in LOITER...")
            self._set_mode("LOITER")
            self.drone_halted = True
        elif not crashed_nodes and self.drone_halted:
            self.get_logger().info("[Orchestrator] All nodes recovered. Resuming mission in GUIDED...")
            self._set_mode("GUIDED")
            self.drone_halted = False

    def _state_machine_loop(self):
        if not self.mavros_state.connected:
            return

        now = self.get_clock().now()
        time_since_service = (now - self.last_service_time).nanoseconds / 1e9

        # Always update header timestamp before publishing
        self.target_pose.header.stamp = now.to_msg()
        self.target_pose.header.frame_id = "map"
        if self.target_pose.pose.orientation.w == 0.0:
            self.target_pose.pose.orientation.w = 1.0

        if self.current_state == "INIT":
            # Throttle async service requests to once every 2 seconds
            if time_since_service > 2.0:
                if not getattr(self, 'stream_rate_requested', False):
                    self.get_logger().info("[Orchestrator] Requesting FCU telemetry stream...")
                    req = StreamRate.Request()
                    req.stream_id = 0  # ALL streams
                    req.message_rate = 10
                    req.on_off = True
                    self.stream_rate_client.call_async(req)
                    self.stream_rate_requested = True
                    self.last_service_time = now
                elif self.mavros_state.mode != "GUIDED":
                    self.get_logger().info("[Orchestrator] Setting mode to GUIDED...")
                    self._set_mode("GUIDED")
                    self.last_service_time = now
                elif not self.mavros_state.armed:
                    self.get_logger().info("[Orchestrator] Sending Arming command...")
                    self._arm()
                    self.last_service_time = now
                else:
                    self.home_x = self.current_pose.pose.position.x
                    self.home_y = self.current_pose.pose.position.y
                    self.get_logger().info(f"[Orchestrator] Armed. Recorded HOME as X={self.home_x:.2f}, Y={self.home_y:.2f}. Sending Takeoff to {self.search_altitude}m")
                    self._takeoff(self.search_altitude)
                    self.current_state = "TAKEOFF"
                    self.last_service_time = now

        elif self.current_state == "TAKEOFF":
            # Check altitude reach threshold
            if self.current_pose.pose.position.z > 1.0:
                self.get_logger().info("[Orchestrator] Altitude reached. Activating Search Grid.")
                self._publish_search_activation(True)
                self.current_state = "SEARCHING"

        elif self.current_state == "SEARCHING":
            if self.drone_halted:
                return
            # Stream active lawnmower target pose from search node
            self.local_pos_pub.publish(self.target_pose)

        elif self.current_state == "TARGET_FOUND":
            self.get_logger().info("[Orchestrator] Halting Search Node.")
            self._publish_search_activation(False)
            
            self.get_logger().info(f"[Orchestrator] Returning to HOME ({self.home_x:.2f}, {self.home_y:.2f}) before precision landing.")
            self.target_pose.pose.position.x = self.home_x
            self.target_pose.pose.position.y = self.home_y
            self.target_pose.pose.position.z = self.search_altitude
            self.current_state = "RETURN_TO_HOME"

        elif self.current_state == "RETURN_TO_HOME":
            self.local_pos_pub.publish(self.target_pose)
            
            # Check if drone reached origin
            dist = math.hypot(self.current_pose.pose.position.x - self.home_x, self.current_pose.pose.position.y - self.home_y)
            if dist < 0.5:
                self.get_logger().info("[Orchestrator] Reached HOME. Triggering Precision Landing.")
                
                self.get_logger().info("[Orchestrator] Releasing Camera from QR Node.")
                self._call_trigger_service(self.release_cam_client)
                
                self._call_trigger_service(self.landing_start_client)
                self.current_state = "LANDING"

        elif self.current_state == "LANDING":
            if not self.mavros_state.armed or self.current_pose.pose.position.z < 0.1:
                self.get_logger().info("[Orchestrator] Touchdown detected. Mission Complete.")
                self.current_state = "DISARMED"
                
        elif self.current_state == "DISARMED":
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
