#!/usr/bin/env python3
"""
search_node.py - Boustrophedon Path Planner with Tree Obstacle Avoidance
                 and Dynamic HMI Priority ROI Preemption/Resumption Stack.

Features:
- Deterministic 100% arena coverage boustrophedon (lawnmower) path synthesis.
- Tree / Hazard exclusion zone pruning (geometric circle collision checks).
- Dynamic ROI Preemption: Stashes global waypoint state on runner input,
  synthesizes dense local micro-grid, and seamlessly resumes global grid if target not found.
"""

import json
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Polygon
from std_msgs.msg import Bool, Float32MultiArray
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import WaypointList, HomePosition
from mavros_msgs.srv import WaypointPull


class SearchNode(Node):
    def __init__(self):
        super().__init__('search_node')

        # Parameters
        self.declare_parameter('recon_x_min', 8.0)
        self.declare_parameter('recon_x_max', 20.0)
        self.declare_parameter('recon_y_min', -6.0)
        self.declare_parameter('recon_y_max', 6.0)
        self.declare_parameter('search_altitude', 3.0)
        self.declare_parameter('lane_spacing', 2.0)
        self.declare_parameter('acceptance_radius', 1.0)
        self.declare_parameter('tree_obstacles_json', '[{"x": 11.0, "y": 1.0, "radius": 2.5}, {"x": 17.0, "y": -2.5, "radius": 2.5}]')

        self.x_min = float(self.get_parameter('recon_x_min').value)
        self.x_max = float(self.get_parameter('recon_x_max').value)
        self.y_min = float(self.get_parameter('recon_y_min').value)
        self.y_max = float(self.get_parameter('recon_y_max').value)
        self.altitude = float(self.get_parameter('search_altitude').value)
        self.lane_spacing = float(self.get_parameter('lane_spacing').value)
        self.acceptance_radius = float(self.get_parameter('acceptance_radius').value)

        # Parse Tree Obstacles
        trees_raw = self.get_parameter('tree_obstacles_json').value
        try:
            self.tree_obstacles = json.loads(trees_raw)
        except Exception as e:
            self.get_logger().warn(f"[Search] Failed to parse tree obstacles JSON: {e}. Using empty obstacle list.")
            self.tree_obstacles = []

        # ROS 2 Publishers
        self.target_waypoint_pub = self.create_publisher(PoseStamped, '/addc/search/waypoint', 10)
        self.search_completed_pub = self.create_publisher(Bool, '/addc/search/completed', 10)

        # ROS 2 Subscribers
        self.activate_sub = self.create_subscription(
            Bool,
            '/addc/search/activate',
            self._activate_callback,
            10
        )
        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._local_pos_callback,
            qos_profile_sensor_data
        )
        self.roi_sub = self.create_subscription(
            Float32MultiArray,
            '/addc/hmi/priority_roi',
            self._hmi_roi_callback,
            10
        )
        self.home_sub = self.create_subscription(
            HomePosition,
            '/mavros/home_position/home',
            self._home_callback,
            qos_profile_sensor_data
        )
        self.mission_sub = self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self._mission_callback,
            qos_profile_sensor_data
        )

        # Fault-Tolerant Heartbeat Publisher
        self.health_pub = self.create_publisher(Bool, '/addc/health/search_node', 10)
        self.health_timer = self.create_timer(0.5, self._publish_health)

        # Waypoint Pull Client
        self.mission_pull_cli = self.create_client(WaypointPull, '/mavros/mission/pull')

        # Internal State
        self.is_active = False
        self.home_pos = None
        self.current_pos: Optional[PoseStamped] = None
        self.waypoints: List[Tuple[float, float, float]] = []
        self.current_idx = 0

        # Current ROI Bounds Memory for Expansion (x_min, x_max, y_min, y_max)
        self.current_roi_bounds: Optional[Tuple[float, float, float, float]] = None
        self.is_in_roi_mode = False

        # Generate Initial Global Lawnmower Grid
        self.generate_global_grid()
        self.get_logger().info(f"[Search] Initialized. Generated {len(self.waypoints)} obstacle-aware search waypoints.")

    def _is_point_in_tree(self, x: float, y: float) -> bool:
        """Checks if coordinate (x, y) violates any tree exclusion buffer."""
        for tree in self.tree_obstacles:
            tx = tree.get('x', 0.0)
            ty = tree.get('y', 0.0)
            r = tree.get('radius', 1.5)
            dist = math.hypot(x - tx, y - ty)
            if dist < r:
                return True
        return False

    def generate_boustrophedon(self, x_min: float, x_max: float, y_min: float, y_max: float,
                               spacing: float, alt: float) -> List[Tuple[float, float, float]]:
        """
        Generates continuous boustrophedon lines, pruning any waypoints that fall inside tree buffers.
        """
        wps: List[Tuple[float, float, float]] = []
        x = x_min
        sweep_up = True

        while x <= x_max:
            y_start = y_min if sweep_up else y_max
            y_end = y_max if sweep_up else y_min

            # Add start point of lane if clear of trees
            if not self._is_point_in_tree(x, y_start):
                wps.append((x, y_start, alt))
            else:
                # Nudge outside buffer
                nudged_y = y_start + (spacing * (1 if sweep_up else -1))
                if y_min <= nudged_y <= y_max:
                    wps.append((x, nudged_y, alt))

            # Add end point of lane if clear of trees
            if not self._is_point_in_tree(x, y_end):
                wps.append((x, y_end, alt))
            else:
                nudged_y = y_end - (spacing * (1 if sweep_up else -1))
                if y_min <= nudged_y <= y_max:
                    wps.append((x, nudged_y, alt))

            x += spacing
            sweep_up = not sweep_up

        return wps

    def generate_global_grid(self):
        self.waypoints = self.generate_boustrophedon(
            self.x_min, self.x_max, self.y_min, self.y_max,
            self.lane_spacing, self.altitude
        )
        self.current_idx = 0
        self.is_in_roi_mode = False

    def _home_callback(self, msg: HomePosition):
        self.home_pos = msg.geo

    def _mission_callback(self, msg: WaypointList):
        if not self.home_pos:
            self.get_logger().warn("[Search] Received mission but no home position yet. Waiting...")
            return
            
        wps = []
        for i, wp in enumerate(msg.waypoints):
            # ArduPilot reserves sequence 0 for the Home Position. Skip it.
            if i == 0:
                continue
                
            if wp.command == 16:  # MAV_CMD_NAV_WAYPOINT
                # Convert GPS to ENU
                R = 6378137.0
                dlat = math.radians(wp.x_lat - self.home_pos.latitude)
                dlon = math.radians(wp.y_long - self.home_pos.longitude)
                ref_lat_rad = math.radians(self.home_pos.latitude)
                
                x = dlon * R * math.cos(ref_lat_rad)
                y = dlat * R
                
                # Force local altitude instead of trusting the raw MAVLink Z (which might be AMSL)
                wps.append((x, y, self.altitude))
                
        if len(wps) > 0 and not self.is_in_roi_mode:
            self.waypoints = wps
            self.current_idx = 0
            self.get_logger().info(f"[Search] Auto-ingested {len(wps)} dynamic global waypoints from GCS Mission upload!")

    def _publish_health(self):
        msg = Bool()
        msg.data = True
        self.health_pub.publish(msg)

    def _activate_callback(self, msg: Bool):
        self.is_active = msg.data
        self.get_logger().info(f"[Search] Active state changed to: {self.is_active}")
        
        if self.is_active:
            # Explicitly demand MAVROS to pull the freshest waypoints from Pixhawk RAM
            self.get_logger().info("[Search] Requesting fresh mission pull from Pixhawk...")
            if self.mission_pull_cli.wait_for_service(timeout_sec=2.0):
                self.mission_pull_cli.call_async(WaypointPull.Request())
            else:
                self.get_logger().warn("[Search] MAVROS mission pull service not available!")
                
            if self.waypoints:
                self._publish_current_waypoint()

    def _hmi_roi_callback(self, msg: Float32MultiArray):
        """
        Dynamic Fallback Trigger:
        msg.data contains [roi_x_min, roi_x_max, roi_y_min, roi_y_max]
        """
        if len(msg.data) < 4:
            self.get_logger().warn("[Search] Invalid ROI data received (expected 4 floats). Ignoring.")
            return

        raw_x_min, raw_x_max, raw_y_min, raw_y_max = msg.data[:4]

        # 1. Enforce strict boundary containment (clamp to arena limits)
        roi_x_min = max(self.x_min, min(self.x_max, raw_x_min))
        roi_x_max = max(self.x_min, min(self.x_max, raw_x_max))
        roi_y_min = max(self.y_min, min(self.y_max, raw_y_min))
        roi_y_max = max(self.y_min, min(self.y_max, raw_y_max))

        if (roi_x_max - roi_x_min) < 1.0 or (roi_y_max - roi_y_min) < 1.0:
            self.get_logger().warn("[Search] Received degenerate/too small ROI. Ignoring.")
            return

        self.get_logger().info(f"[Search] >>> HMI PRIORITY ROI RECEIVED: X[{roi_x_min:.1f}, {roi_x_max:.1f}], Y[{roi_y_min:.1f}, {roi_y_max:.1f}] <<<")

        # 2. Save current ROI bounds for future expansion (no global stash needed)
        self.current_roi_bounds = (roi_x_min, roi_x_max, roi_y_min, roi_y_max)

        # 3. Generate high-density localized micro-grid (1.0m lane spacing for thorough coverage)
        roi_spacing = max(1.0, self.lane_spacing * 0.6)
        roi_waypoints = self.generate_boustrophedon(
            roi_x_min, roi_x_max, roi_y_min, roi_y_max,
            roi_spacing, self.altitude
        )

        if not roi_waypoints:
            self.get_logger().warn("[Search] Could not synthesize valid ROI waypoints due to obstacles.")
            return

        # 4. Activate ROI micro-gridbut 
        self.waypoints = roi_waypoints
        self.current_idx = 0
        self.is_in_roi_mode = True
        self.is_active = True
        self.get_logger().info(f"[Search] Switched to PRIORITY ROI mode with {len(self.waypoints)} micro-waypoints.")

        if self.is_active:
            self._publish_current_waypoint()

    def _expand_roi_search(self):
        """Expands the current ROI bounding box outward and generates a new lawnmower."""
        if not self.current_roi_bounds:
            return
            
        x_min, x_max, y_min, y_max = self.current_roi_bounds
        
        # Expand bounds outward by the standard lane spacing
        expansion = self.lane_spacing
        new_x_min = max(self.x_min, x_min - expansion)
        new_x_max = min(self.x_max, x_max + expansion)
        new_y_min = max(self.y_min, y_min - expansion)
        new_y_max = min(self.y_max, y_max + expansion)
        
        # Check if we have hit the global arena limits
        if new_x_min == x_min and new_x_max == x_max and new_y_min == y_min and new_y_max == y_max:
            self.get_logger().info("[Search] ROI expanded to global limits. Full search exhausted.")
            done_msg = Bool()
            done_msg.data = True
            self.search_completed_pub.publish(done_msg)
            self.is_active = False
            return
            
        self.get_logger().info(f"[Search] >>> ROI EXHAUSTED: Expanding search boundaries to X[{new_x_min:.1f}, {new_x_max:.1f}], Y[{new_y_min:.1f}, {new_y_max:.1f}] <<<")
        self.current_roi_bounds = (new_x_min, new_x_max, new_y_min, new_y_max)
        
        roi_spacing = max(1.0, self.lane_spacing * 0.8)
        self.waypoints = self.generate_boustrophedon(
            new_x_min, new_x_max, new_y_min, new_y_max,
            roi_spacing, self.altitude
        )
        self.current_idx = 0
        self.is_active = True
        if self.is_active:
            self._publish_current_waypoint()

    def _local_pos_callback(self, msg: PoseStamped):
        if not self.is_active or not self.waypoints or self.current_idx >= len(self.waypoints):
            return

        curr_x = msg.pose.position.x
        curr_y = msg.pose.position.y

        target_x, target_y, _ = self.waypoints[self.current_idx]
        dist = math.hypot(curr_x - target_x, curr_y - target_y)

        # Check waypoint acceptance radius
        if dist < self.acceptance_radius:
            self.current_idx += 1
            if self.current_idx < len(self.waypoints):
                self._publish_current_waypoint()
            else:
                # End of current waypoint sequence
                if self.is_in_roi_mode:
                    # ROI exhausted without target spotted -> expand ROI outwards
                    self._expand_roi_search()
                else:
                    # Full arena search complete
                    self.get_logger().info("[Search] Full global boustrophedon sweep completed.")
                    done_msg = Bool()
                    done_msg.data = True
                    self.search_completed_pub.publish(done_msg)
                    self.is_active = False

    def _publish_current_waypoint(self):
        if self.current_idx >= len(self.waypoints):
            return
        x, y, z = self.waypoints[self.current_idx]
        wp = PoseStamped()
        wp.header.stamp = self.get_clock().now().to_msg()
        wp.header.frame_id = "map"
        wp.pose.position.x = float(x)
        wp.pose.position.y = float(y)
        wp.pose.position.z = float(z)
        self.target_waypoint_pub.publish(wp)
        self.get_logger().info(f"[Search] Progress [{self.current_idx+1}/{len(self.waypoints)}]: Heading to X={x:.1f}, Y={y:.1f}, Z={z:.1f} ({'ROI' if self.is_in_roi_mode else 'GLOBAL'})")


def main(args=None):
    rclpy.init(args=args)
    node = SearchNode()
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
