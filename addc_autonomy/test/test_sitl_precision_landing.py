#!/usr/bin/env python3
"""
test_sitl_precision_landing.py - Unit test suite for precision_landing_node
Verifies the Fallback logic: Spiral Search, Washout Lock, and Blind GPS Landing.
"""

import unittest
from unittest.mock import MagicMock, patch
import rclpy
from addc_autonomy.precision_landing_node import PrecisionLandingNode

class TestPrecisionLandingLogic(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        patcher2 = patch('rclpy.node.Node.create_publisher')
        self.mock_pub = patcher2.start()
        self.addCleanup(patcher2.stop)

        patcher3 = patch('rclpy.node.Node.create_subscription')
        self.mock_sub = patcher3.start()
        self.addCleanup(patcher3.stop)
        
        patcher4 = patch('rclpy.node.Node.create_timer')
        self.mock_timer = patcher4.start()
        self.addCleanup(patcher4.stop)
        
        patcher5 = patch('rclpy.node.Node.create_service')
        self.mock_srv = patcher5.start()
        self.addCleanup(patcher5.stop)

        patcher6 = patch('rclpy.node.Node.declare_parameter')
        self.mock_param = patcher6.start()
        self.addCleanup(patcher6.stop)
        
        patcher7 = patch('rclpy.node.Node.get_parameter')
        mock_get_param = patcher7.start()
        from rclpy.parameter import Parameter
        def side_effect_get_param(name):
            if name in ['use_sim_time', 'start_type_description_service']:
                return Parameter(name, Parameter.Type.BOOL, False)
            elif name == 'camera_topic':
                return Parameter(name, Parameter.Type.STRING, '/camera/image_raw')
            else:
                return Parameter(name, Parameter.Type.DOUBLE, 0.5)
        mock_get_param.side_effect = side_effect_get_param
        
        self.addCleanup(patcher7.stop)

        self.node = PrecisionLandingNode()
        # Mock the velocity publisher
        self.node.velocity_pub = MagicMock()
        self.node.landing_cmd_pub = MagicMock()
        self.node.status_pub = MagicMock()

    def test_01_trigger_blind_landing_on_camera_fail(self):
        """Fallback C: If camera fails to init, it should trigger blind land."""
        # Mock hardware failure
        self.node._init_camera = MagicMock(return_value=False)
        
        response = MagicMock()
        self.node._start_landing_cb(None, response)
        
        self.assertTrue(self.node.camera_failed)
        self.node.landing_cmd_pub.publish.assert_called_once()
        self.assertFalse(self.node.is_active)

    def test_02_spiral_search_engagement(self):
        """Fallback A: If pad not found in FOV, engage spiral search."""
        self.node.is_active = True
        self.node.camera_failed = False
        self.node.current_alt = 10.0 # Above washout threshold
        
        # Mock vision to return False (not found)
        self.node._process_vision = MagicMock(return_value=(False, 0.0, 0.0))
        
        self.node._control_loop()
        
        self.assertTrue(self.node.in_spiral_search)
        self.node.velocity_pub.publish.assert_called_once()
        # Check that it commanded spiral velocities, not purely vertical descent
        vel_msg = self.node.velocity_pub.publish.call_args[0][0]
        self.assertEqual(vel_msg.twist.linear.z, 0.0) # Maintains altitude during search

    def test_03_washout_lock_engagement(self):
        """Fallback B: If altitude < threshold, lock descent regardless of vision."""
        self.node.is_active = True
        self.node.camera_failed = False
        self.node.current_alt = 0.4 # Below threshold of 0.5
        
        self.node._control_loop()
        
        self.assertTrue(self.node.in_washout_lock)
        self.node.velocity_pub.publish.assert_called_once()
        # Should command pure vertical descent
        vel_msg = self.node.velocity_pub.publish.call_args[0][0]
        self.assertEqual(vel_msg.twist.linear.x, 0.0)
        self.assertEqual(vel_msg.twist.linear.y, 0.0)
        self.assertLess(vel_msg.twist.linear.z, 0.0)

if __name__ == '__main__':
    unittest.main()
