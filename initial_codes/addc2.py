#!/usr/bin/env python3

import re
import time
import math
import cv2
import numpy as np
from picamera2 import Picamera2

# ROS & MAVROS imports
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, WaypointPush, WaypointClear, WaypointSetCurrent
from mavros_msgs.msg import Waypoint

# 2-Digit QR code pattern
TWO_DIGITS = re.compile(r"^[0-9]{2}$")


class OptimizedMotionQRScanner:
    def __init__(self, record_video=False, output_filename="qr_scan_output.mp4"):
        # Initialize ROS Node
        if not rospy.core.is_initialized():
            rospy.init_node("qr_motion_scanner", anonymous=True)

        # Scanned Payload Publisher
        self.qr_pub = rospy.Publisher("/drone/qr_data", String, queue_size=1)

        # Variables to store the starting location
        self.home_gps = None           # (latitude, longitude, altitude)
        self.current_gps = None
        self.home_local_pose = None    # geometry_msgs/PoseStamped
        self.current_local_pose = None

        # GPS & Local Position Subscribers
        self.gps_sub = rospy.Subscriber(
            "/mavros/global_position/global",
            NavSatFix,
            self._gps_callback,
            queue_size=1
        )
        self.local_pos_sub = rospy.Subscriber(
            "/mavros/local_position/pose",
            PoseStamped,
            self._local_pos_callback,
            queue_size=1
        )

        # Pre-cache persistent MAVROS services
        self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode, persistent=True)
        self.wp_clear_srv = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear, persistent=True)
        self.wp_push_srv = rospy.ServiceProxy("/mavros/mission/push", WaypointPush, persistent=True)
        self.wp_set_curr_srv = rospy.ServiceProxy("/mavros/mission/set_current", WaypointSetCurrent, persistent=True)

        # Camera & Detection Engine Setup
        self.camera = Picamera2()
        self.qr = cv2.QRCodeDetector()[cite: 1]

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))[cite: 1]

        inv_gamma = 1.0 / 0.3[cite: 1]
        self.gamma_lut = np.array(
            [((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]
        ).astype("uint8")[cite: 1]

        self.record_video = record_video[cite: 1]
        self.output_filename = output_filename[cite: 1]

    def _gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude, msg.altitude)
        # Store start location coordinates once valid GPS fix is received
        if self.home_gps is None and msg.latitude != 0.0:
            self.home_gps = self.current_gps
            rospy.loginfo(f"[ROS] Stored Start GPS: Lat {msg.latitude:.6f}, Lon {msg.longitude:.6f}")

    def _local_pos_callback(self, msg):
        self.current_local_pose = msg
        # Store initial Cartesian reference frame coordinate
        if self.home_local_pose is None:
            self.home_local_pose = msg
            rospy.loginfo(f"[ROS] Stored Start Local Pose: X {msg.pose.position.x:.2f}, Y {msg.pose.position.y:.2f}")

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculates spherical distance between two GPS points in meters."""
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def navigate_and_land_at_home(self, cruise_altitude=3.0):
        """
        Pushes a direct 3-step mission:
        1. wp0: Takeoff / Home Baseline
        2. wp1: Direct waypoint to Start Coordinates at 3.0m AGL (MAV_CMD_NAV_WAYPOINT = 16)
        3. wp2: Vertical landing descent at Start Coordinates (MAV_CMD_NAV_LAND = 21)
        Switches to AUTO mode and monitors flight down to touchdown.
        """
        if self.home_gps is None:
            rospy.logerr("[ROS] Cannot navigate: Start GPS coordinates not recorded.")
            return

        lat, lon, base_alt = self.home_gps
        rospy.loginfo(f"[ROS] Directing return to start: Lat {lat:.6f}, Lon {lon:.6f} at {cruise_altitude}m AGL followed by Land.")

        try:
            # 1. Clear previous search pattern waypoints
            rospy.wait_for_service("/mavros/mission/clear", timeout=2.0)
            self.wp_clear_srv()

            # 2. Build Index 0: Home / Takeoff reference (Required by MAVLink)
            wp0 = Waypoint()
            wp0.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp0.command = 16  # MAV_CMD_NAV_WAYPOINT
            wp0.is_current = False
            wp0.autocontinue = True
            wp0.x_lat = lat
            wp0.y_long = lon
            wp0.z_alt = 0.0

            # 3. Build Index 1: Direct cruise waypoint to start coordinate at 3.0m AGL
            wp1 = Waypoint()
            wp1.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp1.command = 16  # MAV_CMD_NAV_WAYPOINT
            wp1.is_current = True
            wp1.autocontinue = True
            wp1.param1 = 0.0  # Hold time (seconds)
            wp1.param2 = 1.5  # Acceptance radius (meters)
            wp1.param3 = 0.0  # Pass through
            wp1.param4 = float('nan')  # Maintain direct target yaw
            wp1.x_lat = lat
            wp1.y_long = lon
            wp1.z_alt = cruise_altitude

            # 4. Build Index 2: Autonomous Landing Waypoint at Start Location
            wp2 = Waypoint()
            wp2.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp2.command = 21  # MAV_CMD_NAV_LAND
            wp2.is_current = False
            wp2.autocontinue = True
            wp2.param1 = 0.0  # Abort altitude (0 = default)
            wp2.param2 = 0.0  # Precision land mode (0 = normal)
            wp2.x_lat = lat
            wp2.y_long = lon
            wp2.z_alt = 0.0  # Touchdown ground level

            # 5. Push 3-item mission to flight controller
            rospy.wait_for_service("/mavros/mission/push", timeout=2.0)
            res = self.wp_push_srv(start_index=0, waypoints=[wp0, wp1, wp2])

            if res.success:
                rospy.loginfo("[ROS] Return and Land mission sequence pushed successfully.")
                
                try:
                    self.wp_set_curr_srv(wp_seq=1)
                except rospy.ServiceException:
                    pass

                # 6. Switch to AUTO mode to execute straight flight & landing
                rospy.wait_for_service("/mavros/set_mode", timeout=2.0)
                self.set_mode_srv(custom_mode="AUTO")
                rospy.loginfo("[ROS] Mode changed to AUTO: Flying direct vector to starting point at 3.0m...")

                # 7. Monitor distance and confirm touchdown
                rate = rospy.Rate(2)
                has_arrived = False

                while not rospy.is_shutdown():
                    if self.current_gps is not None:
                        curr_lat, curr_lon, curr_alt = self.current_gps
                        dist = self.haversine_distance(curr_lat, curr_lon, lat, lon)

                        if not has_arrived:
                            rospy.loginfo(f"[ROS] In Transit -> Distance to Start: {dist:.1f}m")
                            if dist < 2.0:
                                rospy.loginfo("[ROS] Arrived above Start Position! Landing sequence active...")
                                has_arrived = True
                        else:
                            # Verify descent using local pose height if available
                            if self.current_local_pose is not None:
                                current_height = self.current_local_pose.pose.position.z
                                rospy.loginfo(f"[ROS] Landing in progress -> Current Height: {current_height:.2f}m")
                                if current_height <= 0.3:
                                    rospy.loginfo("[ROS] Drone safely touched down at Start Position.")
                                    break
                    rate.sleep()

            else:
                rospy.logerr("[ROS] Failed to push return mission to autopilot.")

        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(f"[ROS] Service execution error during return: {e}")

    def start_camera(self):
        config = self.camera.create_video_configuration(
            main={"size": (960, 540), "format": "BGR888"},
            controls={"FrameRate": 30}
        )[cite: 1]
        self.camera.configure(config)[cite: 1]
        self.camera.start()[cite: 1]
        time.sleep(1.0)[cite: 1]

    def scan_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)[cite: 1]
        retval, points = self.qr.detect(gray)[cite: 1]

        if not retval or points is None:[cite: 1]
            return None, None, 0, 0[cite: 1]

        # Tier 1: Direct Grayscale
        data, _ = self.qr.decode(gray, points)[cite: 1]
        if data and TWO_DIGITS.fullmatch(data.strip()):[cite: 1]
            return data.strip(), points, 0, 0[cite: 1]

        # Tier 2: CLAHE Enhanced
        enhanced = self.clahe.apply(gray)[cite: 1]
        data, _ = self.qr.decode(enhanced, points)[cite: 1]
        if data and TWO_DIGITS.fullmatch(data.strip()):[cite: 1]
            return data.strip(), points, 0, 0[cite: 1]

        # Tier 3: Otsu Thresholding
        _, binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[cite: 1]
        data, _ = self.qr.decode(binary, points)[cite: 1]
        if data and TWO_DIGITS.fullmatch(data.strip()):[cite: 1]
            return data.strip(), points, 0, 0[cite: 1]

        # Tier 4: Gamma Darkening
        darkened = cv2.LUT(gray, self.gamma_lut)[cite: 1]
        data, _ = self.qr.decode(darkened, points)[cite: 1]
        if data and TWO_DIGITS.fullmatch(data.strip()):[cite: 1]
            return data.strip(), points, 0, 0[cite: 1]

        return None, points, 0, 0[cite: 1]

    def run(self):
        self.start_camera()[cite: 1]
        window_name = "RPi Motion QR Scanner"[cite: 1]
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)[cite: 1]

        video_writer = None[cite: 1]
        if self.record_video:[cite: 1]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')[cite: 1]
            video_writer = cv2.VideoWriter(self.output_filename, fourcc, 30.0, (960, 540))[cite: 1]

        print("Scanning for 2-digit QR code (Motion & Distance Optimized)... Press 'q' to quit.")[cite: 1]

        try:
            while not rospy.is_shutdown():
                frame = self.camera.capture_array()[cite: 1]
                if frame is None:[cite: 1]
                    continue[cite: 1]

                h, w = frame.shape[:2][cite: 1]
                cx1, cx2 = int(w * 0.25), int(w * 0.75)[cite: 1]
                cy1, cy2 = int(h * 0.25), int(h * 0.75)[cite: 1]

                digits, points, offset_x, offset_y = self.scan_frame(frame)[cite: 1]

                # Reticle Box
                reticle_color = (0, 255, 0) if digits else (255, 165, 0)[cite: 1]
                cv2.rectangle(frame, (cx1, cy1), (cx2, cy2), reticle_color, 2)[cite: 1]
                cv2.putText(frame, "TARGET RETICLE (MOTION ZONE)", (cx1, cy1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, reticle_color, 1)[cite: 1]

                # QR Bounding Polygon
                if points is not None and len(points) > 0:[cite: 1]
                    pts = points.astype(np.float32)[cite: 1]
                    pts[:, :, 0] += offset_x[cite: 1]
                    pts[:, :, 1] += offset_y[cite: 1]
                    pts = pts.astype(int).reshape((-1, 1, 2))[cite: 1]
                    cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=3)[cite: 1]

                if digits is not None:[cite: 1]
                    cv2.putText(frame, f"SCANNED QR: {digits}", (20, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)[cite: 1]
                    cv2.imshow(window_name, frame)[cite: 1]
                    if video_writer is not None:[cite: 1]
                        video_writer.write(frame)[cite: 1]

                    print(f"\n[+] SUCCESSFUL SCAN: {digits}\n")[cite: 1]

                    # 1. Publish Scanned Payload to ROS
                    self.qr_pub.publish(digits)
                    rospy.loginfo(f"[ROS] Broadcast QR payload: {digits}")

                    # 2. Command Direct Waypoint to Stored Coordinate at 3.0m and Land
                    self.navigate_and_land_at_home(cruise_altitude=3.0)

                    cv2.waitKey(500)[cite: 1]
                    return digits[cite: 1]

                # Status Bar
                cv2.putText(frame, "STATUS: SCANNING... (Press 'q' to quit)", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)[cite: 1]

                if video_writer is not None:[cite: 1]
                    video_writer.write(frame)[cite: 1]

                cv2.imshow(window_name, frame)[cite: 1]
                if cv2.waitKey(1) & 0xFF == ord('q'):[cite: 1]
                    break[cite: 1]

        except (KeyboardInterrupt, rospy.ROSInterruptException):
            return None

        finally:
            self.camera.stop()[cite: 1]
            if video_writer is not None:[cite: 1]
                video_writer.release()[cite: 1]
            cv2.destroyAllWindows()[cite: 1]


if __name__ == "__main__":
    scanner = OptimizedMotionQRScanner(record_video=False)[cite: 1]
    scanner.run()[cite: 1]