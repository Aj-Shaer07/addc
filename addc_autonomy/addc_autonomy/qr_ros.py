#!/usr/bin/env python3
"""
qr_ros.py - ROS 2 Perception Node for ADDC Reconnaissance Mission.

Adapted from QR_Motion_Final.py:
- Preserves exact 4-tier cascaded decode engine (Direct Grayscale -> CLAHE -> Otsu -> Gamma LUT)
- Supports dual camera inputs: Picamera2 (Physical Hardware) vs /camera/image_raw (Gazebo Sim)
- Calculates real normalized centroid error offsets (ex, ey) for proportional visual servoing
- Parameter-driven GUI toggle for headless flight safety (prevents X11/Wayland crashes)
"""

import re
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np

# Standard ROS 2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import Image

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


class QRVisionNode(Node):
    def __init__(self):
        super().__init__('vision_control_node')

        # Declare ROS 2 Parameters
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('enable_debug_window', False)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('record_video', False)
        self.declare_parameter('video_output_path', 'qr_flight_recording.mp4')
        self.declare_parameter('target_fps', 30.0)

        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.enable_gui = self.get_parameter('enable_debug_window').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.record_video = self.get_parameter('record_video').value
        self.video_output_path = self.get_parameter('video_output_path').value
        target_fps = float(self.get_parameter('target_fps').value)

        # ROS 2 Publishers
        # Target Offset: X = Error_X (-1.0 to 1.0), Y = Error_Y (-1.0 to 1.0), Z = Lock Status (1.0 = locked, 0.0 = lost)
        self.target_offset_pub = self.create_publisher(Point, '/addc/vision/target_offset', 10)
        self.decoded_digits_pub = self.create_publisher(String, '/addc/vision/decoded_digits', 10)

        # Core Detection & Decode Engine (Identical to QR_Motion_Final.py)
        self.qr = cv2.QRCodeDetector()
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        inv_gamma = 1.0 / 0.3
        self.gamma_lut = np.array(
            [((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]
        ).astype("uint8")

        # Regex for 2-digit numeric code
        self.two_digits_pattern = re.compile(r"\d.*?\d")

        # Frame Buffer & Threading Lock
        self.latest_frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()
        self.is_running = True
        self.decoded_code: Optional[str] = None
        self.video_writer: Optional[cv2.VideoWriter] = None

        if self.enable_gui:
            self.window_name = "ADDC Vision Control - Target Tracking HUD"
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # Initialize Image Source
        if self.use_sim_time:
            self.get_logger().info(f"[Vision] Running in SIMULATION mode. Subscribing to: {self.camera_topic}")
            self.bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None
            self.image_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self._sim_image_callback,
                10
            )
        else:
            self.get_logger().info("[Vision] Running on HARDWARE. Initializing Picamera2...")
            self._init_picamera2()

        # Timer Callback for Frame Processing Loop
        timer_period = 1.0 / target_fps
        self.processing_timer = self.create_timer(timer_period, self._process_frame_loop)

        self.get_logger().info("[Vision] QRVisionNode initialized successfully.")

    def _init_picamera2(self):
        if not PICAMERA2_AVAILABLE:
            self.get_logger().warn("[Vision] Picamera2 module not found on this system. Operating in headless passive mode.")
            return

        try:
            self.camera = Picamera2()
            config = self.camera.create_video_configuration(
                main={"size": (960, 540), "format": "BGR888"},
                controls={"FrameRate": 30}
            )
            self.camera.configure(config)
            self.camera.start()
            time.sleep(1.0)

            # Dedicated thread to capture frames without stalling the ROS 2 spin loop
            self.cam_thread = threading.Thread(target=self._picamera_capture_worker, daemon=True)
            self.cam_thread.start()
            self.get_logger().info("[Vision] Picamera2 hardware capture thread started.")
        except Exception as e:
            self.get_logger().error(f"[Vision] Failed to initialize Picamera2: {e}")

    def _picamera_capture_worker(self):
        while self.is_running:
            try:
                frame = self.camera.capture_array()
                if frame is not None:
                    with self.frame_lock:
                        self.latest_frame = frame
            except Exception as e:
                self.get_logger().warn(f"[Vision] Camera capture exception: {e}")
                time.sleep(0.05)

    def _sim_image_callback(self, msg: Image):
        """Converts incoming ROS 2 Image messages from Gazebo into OpenCV BGR."""
        try:
            if CV_BRIDGE_AVAILABLE and self.bridge is not None:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                # Fallback manual conversion if cv_bridge is absent
                cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
                if msg.encoding == "rgb8":
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)

            with self.frame_lock:
                self.latest_frame = cv_img
        except Exception as e:
            self.get_logger().error(f"[Vision] Image conversion error: {e}")

    def _extract_digits(self, data: str) -> Optional[str]:
        if not data:
            return None
        all_digits = re.findall(r"\d", data)
        if len(all_digits) >= 2:
            return all_digits[0] + all_digits[1]
        return None

    def scan_frame(self, frame: np.ndarray) -> Tuple[Optional[str], Optional[np.ndarray], float, float]:
        """
        Executes single-pass corner detection followed by the 4-tier cascaded decode engine.
        Returns: (digits, points, error_x, error_y)
        """
        h, w = frame.shape[:2]
        center_x = w / 2.0
        center_y = h / 2.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Single-pass point detection
        retval, points = self.qr.detect(gray)
        if not retval or points is None or len(points) == 0:
            return None, None, 0.0, 0.0

        # Calculate True Normalized Centroid Error (-1.0 to 1.0)
        pts_reshaped = points.reshape(-1, 2)
        centroid_x = float(np.mean(pts_reshaped[:, 0]))
        centroid_y = float(np.mean(pts_reshaped[:, 1]))

        # error_x > 0 means target is to the right of camera center
        # error_y > 0 means target is below camera center
        error_x = (centroid_x - center_x) / center_x
        error_y = (centroid_y - center_y) / center_y

        # Clamp normalized errors within [-1.0, 1.0]
        error_x = max(-1.0, min(1.0, error_x))
        error_y = max(-1.0, min(1.0, error_y))

        # Attempt 1: Direct Grayscale
        data, _ = self.qr.decode(gray, points)
        extracted = self._extract_digits(data)
        if extracted:
            return extracted, points, error_x, error_y

        # Attempt 2: CLAHE Enhanced
        enhanced = self.clahe.apply(gray)
        data, _ = self.qr.decode(enhanced, points)
        extracted = self._extract_digits(data)
        if extracted:
            return extracted, points, error_x, error_y

        # Attempt 3: Otsu Thresholding
        _, binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        data, _ = self.qr.decode(binary, points)
        extracted = self._extract_digits(data)
        if extracted:
            return extracted, points, error_x, error_y

        # Attempt 4: Gamma LUT Darkening (extreme glare fallback)
        darkened = cv2.LUT(gray, self.gamma_lut)
        data, _ = self.qr.decode(darkened, points)
        extracted = self._extract_digits(data)
        if extracted:
            return extracted, points, error_x, error_y

        # Target spotted geometrically, but payload not yet decoded
        return None, points, error_x, error_y

    def _process_frame_loop(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()

        h, w = frame.shape[:2]
        digits, points, error_x, error_y = self.scan_frame(frame)

        # 1. Publish Target Offset & Lock Status
        offset_msg = Point()
        if points is not None:
            offset_msg.x = float(error_x)
            offset_msg.y = float(error_y)
            offset_msg.z = 1.0  # 1.0 = Target Locked
        else:
            offset_msg.x = 0.0
            offset_msg.y = 0.0
            offset_msg.z = 0.0  # 0.0 = Target Lost / Searching
        self.target_offset_pub.publish(offset_msg)

        # 2. Publish Decoded Digits (once verified)
        if digits and (self.decoded_code != digits):
            self.decoded_code = digits
            digits_msg = String()
            digits_msg.data = str(digits)
            self.decoded_digits_pub.publish(digits_msg)
            self.get_logger().info(f"[Vision] >>> SUCCESSFUL 2-DIGIT QR DECODE: {digits} <<<")

        # 3. Optional Video Recording
        if self.record_video:
            if self.video_writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(self.video_output_path, fourcc, 30.0, (w, h))
            self.video_writer.write(frame)

        # 4. Debug Window Rendering (Enabled ONLY during active ground testing)
        if self.enable_gui:
            self._render_gui(frame, digits, points, error_x, error_y)

    def _render_gui(self, frame: np.ndarray, digits: Optional[str], points: Optional[np.ndarray], error_x: float, error_y: float):
        h, w = frame.shape[:2]
        cx1, cx2 = int(w * 0.25), int(w * 0.75)
        cy1, cy2 = int(h * 0.25), int(h * 0.75)

        # Central Target Reticle Box
        reticle_color = (0, 255, 0) if digits else (255, 165, 0)
        cv2.rectangle(frame, (cx1, cy1), (cx2, cy2), reticle_color, 2)
        cv2.putText(frame, "TARGET RETICLE (MOTION ZONE)", (cx1, cy1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, reticle_color, 1)

        # Draw green bounding polygon around QR points
        if points is not None and len(points) > 0:
            pts = points.astype(int).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=3)
            # Centroid crosshair
            c_x = int((w / 2.0) + (error_x * (w / 2.0)))
            c_y = int((h / 2.0) + (error_y * (h / 2.0)))
            cv2.drawMarker(frame, (c_x, c_y), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

        if digits:
            cv2.putText(frame, f"SCANNED QR: {digits}", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

        status_text = f"OFFSET: X={error_x:+.2f}, Y={error_y:+.2f} | LOCK={1 if points is not None else 0}"
        cv2.putText(frame, status_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.is_running = False
        if hasattr(self, 'camera') and self.camera is not None:
            try:
                self.camera.stop()
            except Exception:
                pass
        if self.video_writer is not None:
            self.video_writer.release()
        if self.enable_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QRVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
