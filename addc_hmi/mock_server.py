import json
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

import time
import threading

HMI_STATE = {
    "decoded_digits": "42",
    "last_roi_sent": None,
    "drone_status": "MOCK_SEARCHING",
    "drone_pos_x": 10.0,
    "drone_pos_y": 2.0
}

def simulate_flight():
    while True:
        # Fly diagonally across the map to test all sectors
        HMI_STATE["drone_pos_x"] += 0.5
        HMI_STATE["drone_pos_y"] += 0.2
        if HMI_STATE["drone_pos_x"] > 22.0:
            HMI_STATE["drone_pos_x"] = 6.0
            HMI_STATE["drone_pos_y"] = -8.0
        time.sleep(1.0)

threading.Thread(target=simulate_flight, daemon=True).start()

class MockHMIRequestHandler(BaseHTTPRequestHandler):
    def _send_cors_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def do_OPTIONS(self):
        self.send_response(200)
        self._send_cors_headers()
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/status':
            self.send_response(200)
            self._send_cors_headers()
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(HMI_STATE).encode('utf-8'))
        else:
            self.send_response(404)
            self._send_cors_headers()
            self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/set_roi':
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length)
            try:
                data = json.loads(body.decode('utf-8'))
                roi = data.get('roi')
                if roi and len(roi) == 4:
                    HMI_STATE["last_roi_sent"] = roi
                    print(f"[Mock Server] Received ROI: {roi}")
                    self.send_response(200)
                    self._send_cors_headers()
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "SUCCESS", "roi": roi}).encode('utf-8'))
                    return
            except Exception as e:
                pass
            self.send_response(400)
            self._send_cors_headers()
            self.end_headers()
        else:
            self.send_response(404)
            self._send_cors_headers()
            self.end_headers()

    def log_message(self, format, *args):
        pass

if __name__ == '__main__':
    port = 5000
    server = HTTPServer(('0.0.0.0', port), MockHMIRequestHandler)
    print(f"Starting Mock HMI Server on port {port}...")
    print("This server does NOT require ROS. Use this to test the Flutter app.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down mock server.")
        server.server_close()
