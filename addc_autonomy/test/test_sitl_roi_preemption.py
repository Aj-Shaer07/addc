#!/usr/bin/env python3
"""
test_sitl_roi_preemption.py - Integration test for HMI Bridge HTTP server
and priority ROI payload parsing.
"""

import json
import threading
import time
import unittest
import urllib.request
import urllib.error
from http.server import HTTPServer

from addc_autonomy.hmi_bridge_node import HMIRequestHandler, HMI_STATE


class TestHMIBridgeServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from unittest.mock import MagicMock
        import addc_autonomy.hmi_bridge_node as hmi_module
        hmi_module.HMI_NODE_REF = MagicMock()
        # Bind to dynamic local test port
        cls.test_port = 5899
        cls.server = HTTPServer(('127.0.0.1', cls.test_port), HMIRequestHandler)
        cls.server_thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.server_thread.start()
        time.sleep(0.5)

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()

    def test_01_get_portal_html(self):
        """Verify the root endpoint serves the runner's interactive HMI portal."""
        url = f"http://127.0.0.1:{self.test_port}/"
        with urllib.request.urlopen(url) as response:
            self.assertEqual(response.status, 200)
            content = response.read().decode('utf-8')
            self.assertIn("HMI PORTAL", content)
            self.assertIn("sendROI", content)

    def test_02_post_valid_roi(self):
        """Verify submission of valid sector ROI bounding coordinates."""
        url = f"http://127.0.0.1:{self.test_port}/set_roi"
        payload = json.dumps({"roi": [14.0, 20.0, 0.0, 6.0]}).encode('utf-8')
        req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'}, method='POST')

        with urllib.request.urlopen(req) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode('utf-8'))
            self.assertEqual(data.get("status"), "SUCCESS")
            self.assertEqual(data.get("roi"), [14.0, 20.0, 0.0, 6.0])

    def test_03_post_invalid_roi(self):
        """Verify malformed requests are rejected with HTTP 400."""
        url = f"http://127.0.0.1:{self.test_port}/set_roi"
        payload = json.dumps({"roi": [14.0, 20.0]}).encode('utf-8')  # Incomplete coords
        req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'}, method='POST')

        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(req)
        self.assertEqual(ctx.exception.code, 400)

    def test_04_status_reflection(self):
        """Verify the /status endpoint mirrors the decoded digits in real time."""
        HMI_STATE["decoded_digits"] = "72"
        url = f"http://127.0.0.1:{self.test_port}/status"
        with urllib.request.urlopen(url) as response:
            self.assertEqual(response.status, 200)
            data = json.loads(response.read().decode('utf-8'))
            self.assertEqual(data.get("decoded_digits"), "72")


if __name__ == '__main__':
    unittest.main()
