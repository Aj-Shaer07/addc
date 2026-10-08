#!/usr/bin/env python3
"""
test_sitl_mission_control.py - Unit tests for the Mission Control Orchestrator node.
Verifies the state machine transitions (Takeoff -> Search -> Target Found -> Landing).
"""

import unittest
from unittest.mock import MagicMock, patch
import rclpy
from addc_autonomy.mission_control_node import MissionControlNode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class TestMissionControlLogic(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        patcher1 = patch('rclpy.node.Node.create_publisher')
        self.mock_pub = patcher1.start()
        self.addCleanup(patcher1.stop)

        patcher2 = patch('rclpy.node.Node.create_subscription')
        self.mock_sub = patcher2.start()
        self.addCleanup(patcher2.stop)
        
        patcher3 = patch('rclpy.node.Node.create_timer')
        self.mock_timer = patcher3.start()
        self.addCleanup(patcher3.stop)
        
        patcher4 = patch('rclpy.node.Node.create_client')
        self.mock_client = patcher4.start()
        self.addCleanup(patcher4.stop)

        patcher5 = patch('rclpy.node.Node.declare_parameter')
        self.mock_param = patcher5.start()
        self.addCleanup(patcher5.stop)
        
        patcher6 = patch('rclpy.node.Node.get_parameter')
        mock_get_param = patcher6.start()
        
        from rclpy.parameter import Parameter
        def side_effect_get_param(name):
            if name in ['use_sim_time', 'start_type_description_service']:
                return Parameter(name, Parameter.Type.BOOL, False)
            elif name == 'search_altitude':
                return Parameter(name, Parameter.Type.DOUBLE, 3.0)
            return Parameter(name, Parameter.Type.DOUBLE, 0.0)
        mock_get_param.side_effect = side_effect_get_param
        self.addCleanup(patcher6.stop)

        self.node = MissionControlNode()
        
        # Inject mock publishers and clients for tracking calls
        self.node.local_pos_pub = MagicMock()
        self.node.set_mode_client = MagicMock()
        self.node.arm_client = MagicMock()
        self.node.search_activate_client = MagicMock()
        self.node.release_cam_client = MagicMock()
        self.node.landing_start_client = MagicMock()

    def test_01_init_to_takeoff(self):
        """Verifies transition from INIT to TAKEOFF when armed and in GUIDED mode."""
        self.assertEqual(self.node.current_state, "INIT")
        
        self.node.mavros_state.connected = True
        self.node.mavros_state.mode = "GUIDED"
        self.node.mavros_state.armed = True
        
        self.node._state_machine_loop()
        self.assertEqual(self.node.current_state, "TAKEOFF")

    def test_02_takeoff_to_searching(self):
        """Verifies activation of Search Grid when takeoff altitude is reached."""
        self.node.mavros_state.connected = True
        self.node.current_state = "TAKEOFF"
        
        # Simulate current altitude reaching target 3.0m
        self.node.current_pose.pose.position.z = 2.9
        
        self.node._state_machine_loop()
        self.assertEqual(self.node.current_state, "SEARCHING")
        self.node.search_activate_client.call_async.assert_called_once()

    def test_03_target_found_trigger(self):
        """Verifies vision callback triggers TARGET_FOUND state correctly."""
        self.node.current_state = "SEARCHING"
        self.assertFalse(self.node.target_found)
        
        msg = String()
        msg.data = "42"
        self.node._vision_cb(msg)
        
        self.assertTrue(self.node.target_found)
        self.assertEqual(self.node.current_state, "TARGET_FOUND")

    def test_04_target_found_to_landing(self):
        """Verifies orchestrator releases camera and commands precision landing node."""
        self.node.mavros_state.connected = True
        self.node.current_state = "TARGET_FOUND"
        
        self.node._state_machine_loop()
        
        self.assertEqual(self.node.current_state, "LANDING")
        self.node.search_activate_client.call_async.assert_called_once()
        self.node.release_cam_client.call_async.assert_called_once()
        self.node.landing_start_client.call_async.assert_called_once()

if __name__ == '__main__':
    unittest.main()
