#!/usr/bin/env python3

import re
import time
import cv2
import numpy as np
from picamera2 import Picamera2



class OptimizedMotionQRScanner:
    def __init__(self, record_video=False, output_filename="qr_scan_output.mp4"):
        self.camera = Picamera2()
        self.qr = cv2.QRCodeDetector()
        
        # CLAHE setup from QR_Final.py (excellent contrast for distance & low lighting)
        self.clahe = cv2.createCLAHE(
            clipLimit=2.0,
            tileGridSize=(8, 8)
        )
        
        # Pre-computed Gamma LUT for extreme glare fallback (optimized for RPi 4B CPU)
        inv_gamma = 1.0 / 0.3
        self.gamma_lut = np.array(
            [((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]
        ).astype("uint8")
        
        self.record_video = record_video
        self.output_filename = output_filename

    def start_camera(self):
        # 960x540 BGR888 configuration optimized for RPi 4B performance & OpenCV native rendering
        config = self.camera.create_video_configuration(
            main={
                "size": (960, 540),
                "format": "BGR888"
            },
            controls={
                "FrameRate": 30
            }
        )
        self.camera.configure(config)
        self.camera.start()
        time.sleep(1.0)

    def _extract_digits(self, data):
        if not data:
            return None
        # Find all digit characters in order of appearance (left to right)
        all_digits = re.findall(r"\d", data)
        # Need at least 2 digits; take only the first two, ignore the rest
        if len(all_digits) >= 2:
            return all_digits[0] + all_digits[1]
        return None

    def scan_frame(self, frame):
        """
        Optimized single-pass detection, followed by targeted decoding.
        This drastically reduces CPU load on RPi by avoiding multiple full-frame searches.
        Returns (digits, points, offset_x, offset_y)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 1. Detect QR code location once
        retval, points = self.qr.detect(gray)
        
        if not retval or points is None:
            return None, None, 0, 0

        # We found points! Now try decoding using the known location.
        # This is extremely fast compared to full detection.

        # Attempt 1: Direct Grayscale
        data, _ = self.qr.decode(gray, points)
        extracted = self._extract_digits(data)
        if extracted:
            return extracted, points, 0, 0

        # Attempt 2: CLAHE Enhanced
        enhanced = self.clahe.apply(gray)
        data, _ = self.qr.decode(enhanced, points)
        extracted = self._extract_digits(data)
        if extracted:
            return extracted, points, 0, 0

        # Attempt 3: Otsu Thresholding
        _, binary = cv2.threshold(
            enhanced,
            0,
            255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        data, _ = self.qr.decode(binary, points)
        extracted = self._extract_digits(data)
        if extracted:
            return extracted, points, 0, 0

        # Attempt 4: Pre-computed Gamma Darkening
        darkened = cv2.LUT(gray, self.gamma_lut)
        data, _ = self.qr.decode(darkened, points)
        extracted = self._extract_digits(data)
        if extracted:
            return extracted, points, 0, 0

        # Found a QR code but couldn't decode a valid 2-digit code
        # Still return points so the green box is drawn!
        return None, points, 0, 0

    def run(self):
        self.start_camera()
        window_name = "RPi Motion QR Scanner"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        video_writer = None
        if self.record_video:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(self.output_filename, fourcc, 30.0, (960, 540))

        print("Scanning for 2-digit QR code (Motion & Distance Optimized)... Press 'q' to quit.")

        try:
            while True:
                frame = self.camera.capture_array()
                if frame is None:
                    continue

                h, w = frame.shape[:2]
                cx1, cx2 = int(w * 0.25), int(w * 0.75)
                cy1, cy2 = int(h * 0.25), int(h * 0.75)

                digits, points, offset_x, offset_y = self.scan_frame(frame)

                # Visual HUD: Draw Center Target Reticle Box
                reticle_color = (0, 255, 0) if digits else (255, 165, 0)
                cv2.rectangle(frame, (cx1, cy1), (cx2, cy2), reticle_color, 2)
                cv2.putText(
                    frame,
                    "TARGET RETICLE (MOTION ZONE)",
                    (cx1, cy1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    reticle_color,
                    1
                )

                # Draw green polygon overlay over detected QR code
                if points is not None and len(points) > 0:
                    pts = points.astype(np.float32)
                    pts[:, :, 0] += offset_x
                    pts[:, :, 1] += offset_y
                    pts = pts.astype(int).reshape((-1, 1, 2))
                    cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=3)

                if digits is not None:
                    # Draw detected payload text
                    cv2.putText(
                        frame,
                        f"SCANNED QR: {digits}",
                        (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.2,
                        (0, 255, 0),
                        3
                    )
                    cv2.imshow(window_name, frame)
                    if video_writer is not None:
                        video_writer.write(frame)
                    cv2.waitKey(500)  # Brief visual pause on successful scan
                    print(f"\n[+] SUCCESSFUL SCAN: {digits}\n")
                    return digits

                # HUD Status Bar
                cv2.putText(
                    frame,
                    "STATUS: SCANNING... (Press 'q' to quit)",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

                if video_writer is not None:
                    video_writer.write(frame)

                cv2.imshow(window_name, frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            return None

        finally:
            self.camera.stop()
            if video_writer is not None:
                video_writer.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    scanner = OptimizedMotionQRScanner(record_video=False)
    scanner.run()
