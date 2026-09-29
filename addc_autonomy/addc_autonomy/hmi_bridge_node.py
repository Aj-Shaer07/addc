#!/usr/bin/env python3
"""
hmi_bridge_node.py - Lightweight HMI Web & REST Endpoint over Tailscale.

Features:
- Serves a mobile-optimized web UI for the Ground Operative (runner).
- Allows one-tap Sector ROI submission (or custom bounding coordinates).
- Ingests HTTP POST /set_roi and publishes to /addc/hmi/priority_roi.
- Subscribes to /addc/vision/decoded_digits and displays them live on the runner's screen.
"""

import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

# In-memory shared state between ROS 2 node and HTTP server
HMI_STATE = {
    "decoded_digits": None,
    "last_roi_sent": None,
    "drone_status": "IDLE"
}
HMI_NODE_REF = None


HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ADDC - HMI Operative Portal</title>
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #121212; color: #fff; text-align: center; margin: 0; padding: 20px; }
    h1 { color: #00e5ff; font-size: 22px; margin-bottom: 5px; }
    p.subtitle { color: #888; font-size: 13px; margin-top: 0; }
    .status-card { background: #1e1e1e; border-radius: 12px; padding: 15px; margin: 15px auto; max-width: 400px; border: 1px solid #333; }
    .digits-box { font-size: 48px; font-weight: bold; color: #00ff66; letter-spacing: 10px; margin: 10px 0; }
    .sector-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; max-width: 400px; margin: 20px auto; }
    button.sector-btn { background: #263238; color: #fff; border: 2px solid #00e5ff; border-radius: 10px; padding: 20px 10px; font-size: 16px; font-weight: bold; cursor: pointer; transition: 0.2s; }
    button.sector-btn:active { background: #00e5ff; color: #000; transform: scale(0.98); }
    .ack-msg { color: #ffb300; font-size: 14px; min-height: 20px; margin-top: 10px; }
  </style>
</head>
<body>
  <h1>SAEISS ADDC - HMI PORTAL</h1>
  <p class="subtitle">Ground Operative Priority ROI Dispatch</p>

  <div class="status-card">
    <div style="font-size: 13px; color: #aaa;">DECODED CACHE ACCESS CODE</div>
    <div class="digits-box" id="digits">--</div>
    <div style="font-size: 12px; color: #888;">(Combines with your 2 puzzle digits)</div>
  </div>

  <div style="max-width: 400px; margin: 0 auto; text-align: left; font-size: 14px; color: #ccc;">
    Select estimated cache sector if puzzle completed:
  </div>

  <div class="sector-grid">
    <button class="sector-btn" onclick="sendROI(8, 14, 0, 6, 'Sector 1 (NW)')">SECTOR 1<br><small style="font-weight:normal;color:#aaa;">X[8-14], Y[0-6]</small></button>
    <button class="sector-btn" onclick="sendROI(14, 20, 0, 6, 'Sector 2 (NE)')">SECTOR 2<br><small style="font-weight:normal;color:#aaa;">X[14-20], Y[0-6]</small></button>
    <button class="sector-btn" onclick="sendROI(8, 14, -6, 0, 'Sector 3 (SW)')">SECTOR 3<br><small style="font-weight:normal;color:#aaa;">X[8-14], Y[-6-0]</small></button>
    <button class="sector-btn" onclick="sendROI(14, 20, -6, 0, 'Sector 4 (SE)')">SECTOR 4<br><small style="font-weight:normal;color:#aaa;">X[14-20], Y[-6-0]</small></button>
  </div>

  <div class="ack-msg" id="ack"></div>

  <script>
    async function sendROI(x1, x2, y1, y2, name) {
      document.getElementById('ack').innerText = "Transmitting priority to UAV: " + name + "...";
      try {
        let resp = await fetch('/set_roi', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({roi: [x1, x2, y1, y2]})
        });
        let data = await resp.json();
        document.getElementById('ack').innerText = "✓ UAV Acknowledged: Prioritizing " + name;
      } catch (err) {
        document.getElementById('ack').innerText = "❌ Transmission failed. Check connection.";
      }
    }

    async function pollStatus() {
      try {
        let resp = await fetch('/status');
        let data = await resp.json();
        if (data.decoded_digits) {
          document.getElementById('digits').innerText = data.decoded_digits;
        }
      } catch(e) {}
    }
    setInterval(pollStatus, 1000);
  </script>
</body>
</html>
"""


class HMIRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/' or parsed.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
        elif parsed.path == '/status':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(HMI_STATE).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/set_roi':
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length)
            try:
                data = json.loads(body.decode('utf-8'))
                roi = data.get('roi')
                if roi and len(roi) == 4 and HMI_NODE_REF is not None:
                    HMI_NODE_REF.publish_roi(roi)
                    HMI_STATE["last_roi_sent"] = roi
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "SUCCESS", "roi": roi}).encode('utf-8'))
                    return
            except Exception as e:
                pass
            self.send_response(400)
            self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # Silence default HTTP server console logging
        pass


class HMIBridgeNode(Node):
    def __init__(self):
        super().__init__('hmi_bridge_node')
        global HMI_NODE_REF
        HMI_NODE_REF = self

        self.declare_parameter('http_port', 5000)
        self.http_port = int(self.get_parameter('http_port').value)

        # ROS 2 Publisher & Subscriber
        self.roi_pub = self.create_publisher(Float32MultiArray, '/addc/hmi/priority_roi', 10)
        self.digits_sub = self.create_subscription(
            String,
            '/addc/vision/decoded_digits',
            self._digits_callback,
            10
        )

        # Start Background HTTP Server
        self.server = HTTPServer(('0.0.0.0', self.http_port), HMIRequestHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

        self.get_logger().info(f"[HMI] Tailscale Web Portal active on port {self.http_port}. Open http://<tailscale-ip>:{self.http_port}")

    def publish_roi(self, roi_coords):
        msg = Float32MultiArray()
        msg.data = [float(c) for c in roi_coords]
        self.roi_pub.publish(msg)
        self.get_logger().info(f"[HMI] Broadcasted priority ROI to UAV: {roi_coords}")

    def _digits_callback(self, msg: String):
        HMI_STATE["decoded_digits"] = msg.data
        self.get_logger().info(f"[HMI] Decoded digits received from vision: {msg.data}. Available for runner.")

    def destroy_node(self):
        self.server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HMIBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
