from __future__ import division
import json
import os
import shutil
import tempfile
import unittest

import cv2
import numpy as np
import yaml
from smartcar_calibration.core import Dataset, detect, objects


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix='smartcar-calibration-test-')
        self.ds = Dataset(self.root, 'front', topic='/usb_cam_2/image')
        self.image = np.zeros((480, 640, 3), np.uint8)
        self.matrix = np.array([[600., 0., 320.], [0., 605., 240.], [0., 0., 1.]])
        self.dist = np.array([-.12, .04, .001, -.001, 0.])

    def tearDown(self):
        shutil.rmtree(self.root)

    def points(self, rotation=(.2, -.3, .1), translation=(-.075, -.06, .6)):
        return cv2.projectPoints(objects((8, 6), .025), np.array(rotation, float),
                                 np.array(translation, float), self.matrix, self.dist)[0]

    def test_checkerboard_detection_and_blank_rejection(self):
        board = np.full((440, 560, 3), 255, np.uint8)
        for row in range(7):
            for col in range(9):
                if (row + col) % 2 == 0:
                    board[40 + row * 50:90 + row * 50, 40 + col * 50:90 + col * 50] = 0
        self.assertEqual(detect(board, (8, 6)).shape, (48, 1, 2))
        self.assertIsNone(detect(self.image, (8, 6)))

    def test_stale_missing_duplicate_and_resolution_rejected(self):
        for age, corners in [(3, self.points()), (0, None), (float('nan'), self.points())]:
            with self.assertRaises(ValueError):
                self.ds.add(self.image, corners, age, 'first')
        self.ds.add(self.image, self.points(), 0, 'first')
        with self.assertRaises(ValueError):
            self.ds.add(self.image, self.points(), 0, 'second')
        with self.assertRaises(ValueError):
            self.ds.add(self.image, self.points()[::-1], 0, 'reversed')
        with self.assertRaises(ValueError):
            self.ds.add(np.zeros((600, 800, 3), np.uint8), self.points(), 0, 'third')
        self.assertEqual(self.ds.status()['count'], 1)

    def test_resume_undo_and_camera_isolation(self):
        self.ds.add(self.image, self.points(), 0, 'first')
        resumed = Dataset(self.root, 'front', topic='/usb_cam_2/image')
        self.assertEqual(resumed.status()['count'], 1)
        self.assertEqual(Dataset(self.root, 'rear').status()['count'], 0)
        with self.assertRaises(ValueError):
            Dataset(self.root, 'front', square_m=.02, topic='/usb_cam_2/image')
        resumed.undo()
        self.assertEqual(Dataset(self.root, 'front', topic='/usb_cam_2/image').status()['count'], 0)

    def test_too_few_and_poor_coverage_rejected(self):
        with self.assertRaises(ValueError):
            self.ds.calibrate()
        self.ds.meta['image_size'] = [640, 480]
        self.ds.meta['samples'] = [{'corners': self.points().reshape(-1, 2).tolist()}] * 20
        with self.assertRaises(ValueError):
            self.ds.calibrate()

    def test_synthetic_intrinsics_recovery_yaml_and_quality_gate(self):
        rng = np.random.RandomState(19)
        for i in range(500):
            z = rng.uniform(.36, .75)
            center = np.array([rng.uniform(130, 510), rng.uniform(100, 380)])
            translation = ((center[0] - 320) * z / 600 - .0875,
                           (center[1] - 240) * z / 605 - .0625, z)
            points = self.points(rng.uniform(-.55, .55, 3), translation)
            points += rng.normal(0, .08, points.shape).astype(np.float32)
            try:
                self.ds.add(self.image, points, .1, str(i))
            except ValueError:
                continue
            if self.ds.status()['count'] == 35:
                break
        report, matrix, distortion = self.ds.calibrate()
        self.assertTrue(report['automatic_checks_passed'], report)
        self.assertLess(report['rms_px'], .2)
        np.testing.assert_allclose(matrix[:2, :2], self.matrix[:2, :2], atol=3)
        self.assertLess(abs(distortion.reshape(-1)[0] - self.dist[0]), .02)
        with open(os.path.join(report['result_directory'], 'camera.yaml')) as stream:
            saved = yaml.safe_load(stream)
        self.assertEqual(saved['camera_name'], 'smartcar_front')
        self.assertEqual(saved['distortion_model'], 'plumb_bob')
        self.assertEqual(saved['distortion_coefficients']['cols'], 5)
        self.assertEqual(saved['projection_matrix']['cols'], 4)
        with open(os.path.join(report['result_directory'], 'dataset.json')) as stream:
            self.assertEqual(len(json.load(stream)['samples']), 35)
        # Corrupt one accepted observation: retain an inspectable candidate, fail the quality gate.
        self.ds.meta['samples'][0]['corners'][10][0] += 40
        bad, _, _ = self.ds.calibrate()
        self.assertFalse(bad['automatic_checks_passed'])
        self.assertNotEqual(bad['result_directory'], report['result_directory'])


if __name__ == '__main__':
    unittest.main()
