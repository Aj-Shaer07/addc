#!/usr/bin/env python3
"""
precision_landing_node.py - Dedicated Pad Recovery and Precision Landing module.

Features:
- Initializes Picamera2 exclusively upon arriving at home location.
- Falls back to GPS landing if camera fails to initialize.
- Executes Visual Servoing (PID) to lock onto landing pad.
- If pad not found in FOV -> Fallback A: Localized Spiral Search.
- If pad lost during descent due to wash-out below threshold -> Fallback B: Blind descend lock.
"""

import math
import time
import threading
from typing import Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from mavros_msgs.msg import State
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False


class PrecisionLandingNode(Node):
    def __init__(self):
        super().__init__('precision_landing_node')

        # Parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('enable_debug_window', False)
        self.declare_parameter('landing_alt_threshold', 0.5) # meters (Washout lock threshold)
        self.declare_parameter('descent_speed', 0.3) # m/s
        self.declare_parameter('spiral_radius_max', 2.0)
        
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.washout_threshold = float(self.get_parameter('landing_alt_threshold').value)
        self.descent_speed = float(self.get_parameter('descent_speed').value)
        self.spiral_radius_max = float(self.get_parameter('spiral_radius_max').value)
        self.enable_gui = self.get_parameter('enable_debug_window').value

        if self.enable_gui:
            cv2.namedWindow("Precision Landing HUD", cv2.WINDOW_NORMAL)

        # State Variables
        self.is_active = False
        self.camera_initialized = False
        self.camera_failed = False
        self.current_alt = 10.0
        self.current_local_pos = None
        
        self.pad_locked = False
        self.last_seen_time = 0.0
        
        # Spiral search state
        self.in_spiral_search = False
        self.spiral_start_pos = None
        self.spiral_angle = 0.0
        
        # Washout State
        self.in_washout_lock = False

        # ROS 2 Subscribers
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._local_pos_callback, qos_profile_sensor_data)
        self.state_sub = self.create_subscription(State, '/mavros/state', self._state_callback, qos_profile_sensor_data)
        
        # ROS 2 Publishers
        self.velocity_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos_profile_sensor_data)
        self.landing_cmd_pub = self.create_publisher(String, '/addc/mission/land_cmd', 10)
        self.status_pub = self.create_publisher(String, '/addc/landing/status', 10)

        # ROS 2 Services
        self.start_srv = self.create_service(Trigger, '/addc/landing/start', self._start_landing_cb)

        # Vision Setup
        self.bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        self.camera = None
        self.cam_thread = None
        self.image_sub = None

        # Control Loop Timer
        self.control_timer = self.create_timer(0.1, self._control_loop) # 10Hz
        self.get_logger().info("[Landing] Precision Landing Node Initialized (Idle).")

    def _start_landing_cb(self, request, response):
        """Triggered by orchestrator when drone reaches home X,Y."""
        self.get_logger().info("[Landing] Orchestrator triggered precision landing sequence.")
        self.is_active = True
        self.in_spiral_search = False
        self.in_washout_lock = False
        self.spiral_angle = 0.0
        self.spiral_start_pos = self.current_local_pos
        self.pad_locked = False
        
        # Attempt to initialize camera
        success = self._init_camera()
        if not success:
            self.camera_failed = True
            self.get_logger().error("[Landing] CAMERA FALLBACK: Hardware failed to initialize. Triggering blind GPS landing.")
            self._trigger_blind_land()
            response.success = True
            response.message = "Fallback: Blind GPS Landing initiated."
            return response

        response.success = True
        response.message = "Camera initialized. Beginning visual precision descent."
        return response

    def _init_camera(self) -> bool:
        if self.use_sim_time:
            self.get_logger().info(f"[Landing] SIMULATION: Subscribing to {self.camera_topic}")
            self.image_sub = self.create_subscription(Image, self.camera_topic, self._sim_image_callback, 10)
            self.camera_initialized = True
            return True
            
        if not PICAMERA2_AVAILABLE:
            self.get_logger().warn("[Landing] Picamera2 not found. Simulating camera failure.")
            return False

        try:
            self.camera = Picamera2()
            config = self.camera.create_video_configuration(main={"size": (640, 480), "format": "BGR888"})
            self.camera.configure(config)
            self.camera.start()
            time.sleep(1.0)

            self.cam_thread = threading.Thread(target=self._picamera_worker, daemon=True)
            self.cam_thread.start()
            self.camera_initialized = True
            self.get_logger().info("[Landing] Picamera2 hardware started successfully.")
            return True
        except Exception as e:
            self.get_logger().error(f"[Landing] Failed to start Picamera2: {e}")
            return False

    def _picamera_worker(self):
        while self.is_active and self.camera_initialized:
            try:
                frame = self.camera.capture_array()
                if frame is not None:
                    with self.frame_lock:
                        self.latest_frame = frame
            except Exception as e:
                time.sleep(0.1)

    def _sim_image_callback(self, msg: Image):
        if not self.is_active:
            return
        try:
            if CV_BRIDGE_AVAILABLE and self.bridge is not None:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            with self.frame_lock:
                self.latest_frame = cv_img
        except Exception:
            pass

    def _local_pos_callback(self, msg: PoseStamped):
        self.current_local_pos = msg.pose.position
        self.current_alt = msg.pose.position.z

    def _state_callback(self, msg: State):
        pass

    def _process_vision(self) -> Tuple[bool, float, float]:
        """Simple mock visual detector for the landing pad (e.g., color threshold)."""
        # SITL Robust Fallback: Mock the vision tracking offsets using odometry directly to the EKF local origin (0, 0)
        if getattr(self, 'use_sim_time', False) and self.current_local_pos is not None:
            # P-controller expects error, so distance to the launch pad (0,0 in local frame)
            err_x = 0.0 - self.current_local_pos.x
            err_y = 0.0 - self.current_local_pos.y
            
            # Normalize to [-1.0, 1.0] mimicking image pixel offset ratios
            err_x = max(-1.0, min(1.0, err_x))
            err_y = max(-1.0, min(1.0, err_y))
            
            return True, err_x, err_y

        with self.frame_lock:
            if self.latest_frame is None:
                return False, 0.0, 0.0
            frame = self.latest_frame.copy()

        # Physical Hardware Vision Processing: Color-Agnostic Shape Detector
        h, w = frame.shape[:2]
        center_x = w / 2.0
        center_y = h / 2.0

        # Convert to Grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply slight blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Canny Edge Detection (Finds sharp transitions regardless of color)
        edges = cv2.Canny(blurred, 50, 150)

        # Dilate edges to close gaps in the square's outline
        kernel = np.ones((5,5), np.uint8)
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_cnt = None
        max_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 800:  # Ignore tiny specks of white
                # Approximate polygon to check for square-ish shapes
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                
                # Check for 4 corners (or close to it)
                if len(approx) >= 4:
                    x, y, w_box, h_box = cv2.boundingRect(approx)
                    aspect_ratio = float(w_box) / h_box
                    
                    # Ensure it's roughly a square and is the largest white square we see
                    if 0.7 <= aspect_ratio <= 1.3 and area > max_area:
                        max_area = area
                        best_cnt = cnt

        if best_cnt is not None:
            # Calculate moments for the true centroid of the white square
            M = cv2.moments(best_cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Normalize error from -1.0 to 1.0 based on camera center
                err_x = (cx - center_x) / center_x
                err_y = (cy - center_y) / center_y
                
                # Clamp values to prevent aggressive over-correction
                err_x = max(-1.0, min(1.0, err_x))
                err_y = max(-1.0, min(1.0, err_y))
                
                # Draw debug info
                if self.enable_gui:
                    cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.putText(frame, f"LOCKED: errX={err_x:.2f} errY={err_y:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                if self.enable_gui:
                    cv2.imshow("Precision Landing HUD", frame)
                    cv2.waitKey(1)
                
                return True, err_x, err_y

        if self.enable_gui:
            cv2.putText(frame, "NO PAD DETECTED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("Precision Landing HUD", frame)
            cv2.waitKey(1)

        # Pad not found in this physical frame
        return False, 0.0, 0.0

    def _trigger_blind_land(self):
        msg = String()
        msg.data = "LAND"
        self.landing_cmd_pub.publish(msg)
        self.publish_status("FALLBACK_BLIND_LAND")
        self.is_active = False

    def publish_status(self, stat: str):
        m = String()
        m.data = stat
        self.status_pub.publish(m)

    def _control_loop(self):
        if not self.is_active or self.camera_failed:
            return

        # 1. Washout Lock Fallback
        if self.current_alt < self.washout_threshold:
            if not self.in_washout_lock:
                self.get_logger().info(f"[Landing] Alt {self.current_alt:.1f}m < threshold. Engaging WASHOUT LOCK.")
                self.in_washout_lock = True
                self.publish_status("WASHOUT_LOCK")
            
            # Send pure vertical descent
            vel = TwistStamped()
            vel.twist.linear.x = 0.0
            vel.twist.linear.y = 0.0
            vel.twist.linear.z = -self.descent_speed
            self.velocity_pub.publish(vel)
            
            # If very close to ground, trigger disarm/land
            if self.current_alt < 0.1:
                self._trigger_blind_land()
            return

        # 2. Vision Processing
        found, err_x, err_y = self._process_vision()
        now = self.get_clock().now().nanoseconds / 1e9

        if found:
            self.pad_locked = True
            self.last_seen_time = now
            self.in_spiral_search = False
            self.publish_status("VISUAL_TRACKING")
            
            vel = TwistStamped()
            vel.twist.linear.x = float(err_x * 0.5)  # P-controller
            vel.twist.linear.y = float(err_y * 0.5)
            vel.twist.linear.z = -self.descent_speed
            self.velocity_pub.publish(vel)
        else:
            # Pad lost or not yet found
            if self.pad_locked and (now - self.last_seen_time < 2.0):
                # Temporary loss, hover and wait
                vel = TwistStamped()
                vel.twist.linear.x = 0.0
                vel.twist.linear.y = 0.0
                vel.twist.linear.z = 0.0
                self.velocity_pub.publish(vel)
                self.publish_status("HOVER_SEARCHING")
            else:
                # 3. Localized Spiral Search Fallback
                self.pad_locked = False
                if not self.in_spiral_search:
                    self.get_logger().info("[Landing] Pad not in FOV. Engaging SPIRAL SEARCH.")
                    self.in_spiral_search = True
                    self.spiral_angle = 0.0
                    if self.current_local_pos:
                        self.spiral_start_pos = self.current_local_pos
                
                self.publish_status("SPIRAL_SEARCH")
                # Expand radius gradually
                radius = min(self.spiral_radius_max, 0.5 + (self.spiral_angle * 0.05))
                
                # Spiral velocity vectors
                vx = -radius * math.sin(self.spiral_angle) * 0.5
                vy = radius * math.cos(self.spiral_angle) * 0.5
                
                vel = TwistStamped()
                vel.twist.linear.x = float(vx)
                vel.twist.linear.y = float(vy)
                vel.twist.linear.z = 0.0 # Maintain altitude during search
                self.velocity_pub.publish(vel)
                
                self.spiral_angle += 0.2

    def destroy_node(self):
        self.is_active = False
        if self.camera is not None:
            try:
                self.camera.stop()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandingNode()
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
