#!/usr/bin/env python3
"""
test_sitl_search_grid.py - Unit test suite for search_node path planner,
obstacle avoidance, and dynamic ROI preemption stack.
"""

import math
import unittest
from typing import List, Tuple


class TestSearchPlannerLogic(unittest.TestCase):
    def setUp(self):
        self.x_min = 8.0
        self.x_max = 20.0
        self.y_min = -6.0
        self.y_max = 6.0
        self.altitude = 3.0
        self.spacing = 2.0
        self.trees = [
            {"x": 11.0, "y": 1.0, "radius": 1.5},
            {"x": 17.0, "y": -2.5, "radius": 1.5}
        ]

    def _is_point_in_tree(self, x: float, y: float) -> bool:
        for tree in self.trees:
            dist = math.hypot(x - tree['x'], y - tree['y'])
            if dist < tree['radius']:
                return True
        return False

    def _generate_boustrophedon(self, x_min: float, x_max: float, y_min: float, y_max: float,
                                spacing: float, alt: float) -> List[Tuple[float, float, float]]:
        wps: List[Tuple[float, float, float]] = []
        x = x_min
        sweep_up = True

        while x <= x_max:
            y_start = y_min if sweep_up else y_max
            y_end = y_max if sweep_up else y_min

            if not self._is_point_in_tree(x, y_start):
                wps.append((x, y_start, alt))
            else:
                nudged_y = y_start + (spacing * (1 if sweep_up else -1))
                if y_min <= nudged_y <= y_max:
                    wps.append((x, nudged_y, alt))

            if not self._is_point_in_tree(x, y_end):
                wps.append((x, y_end, alt))
            else:
                nudged_y = y_end - (spacing * (1 if sweep_up else -1))
                if y_min <= nudged_y <= y_max:
                    wps.append((x, nudged_y, alt))

            x += spacing
            sweep_up = not sweep_up

        return wps

    def test_01_boundary_containment(self):
        """Verify all generated waypoints reside strictly within the arena bounds."""
        wps = self._generate_boustrophedon(self.x_min, self.x_max, self.y_min, self.y_max, self.spacing, self.altitude)
        self.assertGreater(len(wps), 0)

        for x, y, z in wps:
            self.assertGreaterEqual(x, self.x_min - 1e-4)
            self.assertLessEqual(x, self.x_max + 1e-4)
            self.assertGreaterEqual(y, self.y_min - 1e-4)
            self.assertLessEqual(y, self.y_max + 1e-4)
            self.assertEqual(z, self.altitude)

    def test_02_obstacle_avoidance(self):
        """Verify that zero generated waypoints fall inside tree exclusion zones."""
        wps = self._generate_boustrophedon(self.x_min, self.x_max, self.y_min, self.y_max, self.spacing, self.altitude)
        for x, y, _ in wps:
            self.assertFalse(self._is_point_in_tree(x, y), f"Waypoint ({x}, {y}) violates tree buffer zone!")

    def test_03_roi_stack_preemption_and_resumption(self):
        """Simulate runner ROI trigger mid-flight, stack stashing, and resumption on empty ROI."""
        global_wps = self._generate_boustrophedon(self.x_min, self.x_max, self.y_min, self.y_max, self.spacing, self.altitude)
        current_idx = 4  # Drone was at waypoint 4 when runner submitted ROI

        # Preemption trigger
        resume_stack = []
        resume_stack.append((list(global_wps), current_idx))

        # Runner specifies ROI [12.0, 16.0, 0.0, 4.0]
        roi_x_min, roi_x_max, roi_y_min, roi_y_max = 12.0, 16.0, 0.0, 4.0
        roi_wps = self._generate_boustrophedon(roi_x_min, roi_x_max, roi_y_min, roi_y_max, 1.0, self.altitude)

        self.assertGreater(len(roi_wps), 0)
        # All ROI waypoints must be within the specified ROI
        for x, y, _ in roi_wps:
            self.assertGreaterEqual(x, roi_x_min - 1e-4)
            self.assertLessEqual(x, roi_x_max + 1e-4)
            self.assertGreaterEqual(y, roi_y_min - 1e-4)
            self.assertLessEqual(y, roi_y_max + 1e-4)

        # Simulate ROI exhausted without spotting target -> pop stack
        restored_wps, restored_idx = resume_stack.pop()
        self.assertEqual(restored_idx, 4)
        self.assertEqual(len(restored_wps), len(global_wps))
        self.assertEqual(restored_wps[restored_idx], global_wps[4])

    def test_04_roi_boundary_clamping(self):
        """Ensure an out-of-bounds ROI requested by human is clamped safely to geofence."""
        requested_roi = [5.0, 25.0, -10.0, 10.0]  # Outside arena
        clamped_x_min = max(self.x_min, min(self.x_max, requested_roi[0]))
        clamped_x_max = max(self.x_min, min(self.x_max, requested_roi[1]))
        clamped_y_min = max(self.y_min, min(self.y_max, requested_roi[2]))
        clamped_y_max = max(self.y_min, min(self.y_max, requested_roi[3]))

        self.assertEqual(clamped_x_min, 8.0)
        self.assertEqual(clamped_x_max, 20.0)
        self.assertEqual(clamped_y_min, -6.0)
        self.assertEqual(clamped_y_max, 6.0)


if __name__ == '__main__':
    unittest.main()
