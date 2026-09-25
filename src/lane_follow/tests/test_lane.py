import json
import os
import unittest
import cv2
import numpy as np
from lane_follow.vision import LaneDetector, permit_motion, check_scan
from lane_follow.control import SteeringController

CFG = json.load(open(os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
                                'config', 'lane_follow.json')))


class LaneTests(unittest.TestCase):
    def test_controller_damps_approach(self):
        controller = SteeringController(CFG)
        result = None
        for i, error in enumerate([-.12, -.12, -.10, -.08, -.06, -.04, -.02, 0]):
            result = controller.update(dict(valid=True, error=error), 100+i*.1)
        self.assertGreater(result['d_term'], 0)
        self.assertGreater(result['raw_steering'], result['p_term'])
        self.assertLessEqual(abs(result['d_term']), CFG['derivative_limit'])

    def test_controller_rejects_invalid_and_time_reversal(self):
        controller = SteeringController(CFG)
        good = dict(valid=True, error=.1)
        self.assertIsNotNone(controller.update(good, 100))
        self.assertIsNone(controller.update(good, 99))
        self.assertIsNone(controller.update(dict(valid=False), 101))
        self.assertIsNone(controller.update(dict(valid=True, error=float('nan')), 102))

    def test_controller_rejects_single_frame_spike(self):
        controller = SteeringController(CFG)
        for i, error in enumerate([.1, .1, .1, .5, .1]):
            result = controller.update(dict(valid=True, error=error), 100+i*.1)
            if i >= 2:
                self.assertAlmostEqual(result['filtered_error'], .1)
                self.assertAlmostEqual(result['raw_steering'], 2.)

    def image(self, shift=0):
        image = np.full((480, 640, 3), 60, np.uint8)
        cv2.line(image, (240+shift, 240), (50+shift, 430), (255, 255, 255), 8)
        cv2.line(image, (400+shift, 240), (590+shift, 430), (255, 255, 255), 8)
        return image

    def test_direction_and_limit(self):
        detector = LaneDetector(CFG)
        for shift in [-45, 0, 45]:
            result, unused = detector.detect(self.image(shift))
            self.assertTrue(result['valid'], result)
            self.assertLessEqual(abs(result['raw_steering']), CFG['max_steering'])
            if shift:
                self.assertGreater(result['raw_steering']*shift, 0)
            else:
                self.assertLess(abs(result['raw_steering']), 1)

    def test_missing_and_horizontal(self):
        detector = LaneDetector(CFG)
        empty = np.zeros((480, 640, 3), np.uint8)
        self.assertFalse(detector.detect(empty)[0]['valid'])
        cv2.line(empty, (0, 330), (639, 330), (255, 255, 255), 10)
        self.assertFalse(detector.detect(empty)[0]['valid'])
        image = self.image()
        image[:, 320:] = 60
        self.assertFalse(detector.detect(image)[0]['valid'])

    def test_stop_gates(self):
        scan = dict(received=100., stamp=99.9, ranges=[2.]*1440,
                    angle_min=0., angle_increment=2*np.pi/1440)
        self.assertIsNone(permit_motion(100, 103, 99.95, 99.95, scan, CFG))
        self.assertEqual(permit_motion(103, 103, 103, 103, scan, CFG), 'duration_limit')
        self.assertEqual(permit_motion(100, 103, 99, 100, scan, CFG), 'camera_stale')
        self.assertEqual(permit_motion(100, 103, 100, 99, scan, CFG), 'lane_inference_stale')
        self.assertEqual(permit_motion(101, 103, 101, 101, scan, CFG), 'lidar_missing_or_stale')
        scan['ranges'][:5] = [.5]*5
        self.assertEqual(check_scan(scan, 100, CFG), 'front_obstacle')
        scan['ranges'] = [float('nan')]*1440
        self.assertEqual(check_scan(scan, 100, CFG), 'insufficient_front_lidar_coverage')


if __name__ == '__main__':
    unittest.main()
