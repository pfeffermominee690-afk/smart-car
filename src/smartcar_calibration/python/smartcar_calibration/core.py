# -*- coding: utf-8 -*-
from __future__ import division

import hashlib
import json
import os
import time
import uuid

import cv2
import numpy as np
import yaml

MIN_SAMPLES = 20


def atomic_write(path, data):
    temporary = path + '.tmp'
    with open(temporary, 'wb') as stream:
        stream.write(data.encode('utf-8') if not isinstance(data, bytes) else data)
        stream.flush()
        os.fsync(stream.fileno())
    os.rename(temporary, path)  # Ubuntu/POSIX: replace atomically.


def dump_json(path, data):
    atomic_write(path, json.dumps(data, indent=2, sort_keys=True, allow_nan=False))


def detect(image, board):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Reduce detection cost on the Jetson; refine and store corners at original resolution.
    scale = min(1.0, 480.0 / gray.shape[1])
    small = cv2.resize(gray, (int(round(gray.shape[1] * scale)), int(round(gray.shape[0] * scale)))) if scale < 1 else gray
    found, corners = cv2.findChessboardCorners(
        small, tuple(board), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
    if not found:
        return None
    corners[:, :, 0] *= float(gray.shape[1]) / small.shape[1]
    corners[:, :, 1] *= float(gray.shape[0]) / small.shape[0]
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))


def objects(board, square_m):
    points = np.zeros((board[0] * board[1], 3), np.float32)
    points[:, :2] = np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2) * square_m
    return points


def coverage(samples, size):
    if not samples:
        return {'center_span': [0, 0], 'corner_span': [0, 0], 'size_ratio': 1.0}
    points = np.array([s['corners'] for s in samples], dtype=float).reshape(len(samples), -1, 2)
    norm = points / np.array(size)
    areas = [abs(cv2.contourArea(cv2.convexHull(p.astype(np.float32)))) for p in norm]
    return {'center_span': np.ptp(norm.mean(axis=1), axis=0).tolist(),
            'corner_span': np.ptp(norm.reshape(-1, 2), axis=0).tolist(),
            'size_ratio': float(np.sqrt(max(areas) / max(min(areas), 1e-12)))}


class Dataset(object):
    def __init__(self, root, camera, board=(8, 6), square_m=0.025, topic=''):
        if camera not in ('front', 'rear'):
            raise ValueError('Camera must be front or rear')
        if len(board) != 2 or min(board) < 3 or max(board) > 30 or not 0 < square_m < 1:
            raise ValueError('Invalid board dimensions or square size')
        self.directory = os.path.join(root, camera)
        self.path = os.path.join(self.directory, 'dataset.json')
        self.meta = {'schema': 1, 'camera': camera, 'board': list(board), 'square_m': square_m,
                     'topic': topic, 'image_size': None, 'samples': []}
        if not os.path.isdir(self.directory):
            os.makedirs(self.directory)
        if os.path.exists(self.path):
            with open(self.path) as stream:
                old = json.load(stream)
            for field in ('schema', 'camera', 'board', 'square_m', 'topic'):
                if old[field] != self.meta[field]:
                    raise ValueError('Existing dataset has different ' + field + '; use a new session')
            self.meta = old
            for sample in old['samples']:
                if not os.path.isfile(os.path.join(self.directory, sample['file'])):
                    raise ValueError('Missing sample image: ' + sample['file'])

    def status(self):
        return {'count': len(self.meta['samples']), 'minimum': MIN_SAMPLES,
                'coverage': coverage(self.meta['samples'], self.meta['image_size']),
                'directory': self.directory, 'image_size': self.meta['image_size']}

    def add(self, image, corners, age, stamp):
        if not np.isfinite(age) or age < 0 or age > 2.0:
            raise ValueError('Image is stale; wait for a live camera frame')
        if corners is None:
            raise ValueError('Full %dx%d board not detected; move board into view' % tuple(self.meta['board']))
        size = [int(image.shape[1]), int(image.shape[0])]
        points = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
        if points.shape != (self.meta['board'][0] * self.meta['board'][1], 2) or not np.isfinite(points).all():
            raise ValueError('Invalid corner coordinates')
        if (points < 4).any() or (points > np.array(size) - 5).any():
            raise ValueError('Board too close to image boundary')
        if self.meta['image_size'] and self.meta['image_size'] != size:
            raise ValueError('Resolution changed; start a new session')
        # Detect translation, scale and perspective changes in normalized corner positions.
        # Accepting many identical views gives a misleadingly low reprojection error.
        for sample in self.meta['samples']:
            if sample['stamp'] == stamp:
                raise ValueError('This exact frame has already been captured')
            previous = np.array(sample['corners']).reshape(-1, 2)
            distance = min(np.sqrt(np.mean(((points - p) / size) ** 2))
                           for p in (previous, previous[::-1]))
            if distance < 0.025:
                raise ValueError('Pose too similar; change position, distance or tilt')
        filename = 'image-' + uuid.uuid4().hex + '.png'
        if not cv2.imwrite(os.path.join(self.directory, filename), image):
            raise IOError('Unable to save sample image')
        self.meta['image_size'] = size
        self.meta['samples'].append({'file': filename, 'corners': points.tolist(),
                                     'stamp': stamp, 'captured_at': time.time()})
        dump_json(self.path, self.meta)

    def undo(self):
        if not self.meta['samples']:
            raise ValueError('No sample to undo')
        self.meta['samples'].pop()  # Keep original PNG for audit; manifest selects active samples.
        if not self.meta['samples']:
            self.meta['image_size'] = None
        dump_json(self.path, self.meta)

    def calibrate(self):
        samples = self.meta['samples']
        if len(samples) < MIN_SAMPLES:
            raise ValueError('Need at least %d diverse samples' % MIN_SAMPLES)
        size = tuple(self.meta['image_size'])
        cov = coverage(samples, size)
        if min(cov['center_span']) < 0.25 or min(cov['corner_span']) < 0.60 or cov['size_ratio'] < 1.3:
            raise ValueError('Insufficient coverage: move board left/right/up/down and near/far')
        obj = objects(self.meta['board'], self.meta['square_m'])
        img = [np.array(s['corners'], np.float32).reshape(-1, 1, 2) for s in samples]
        rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            [obj.copy() for _ in samples], img, size, None, None)
        if not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or not np.isfinite(rms):
            raise ValueError('Non-finite calibration; collect a better dataset')
        normals = [cv2.Rodrigues(r)[0][:, 2] for r in rvecs]
        tilt = max(np.degrees(np.arccos(np.clip(np.dot(a, b), -1, 1)))
                   for a in normals for b in normals)
        errors = []
        for sample, observed, rvec, tvec in zip(samples, img, rvecs, tvecs):
            projected = cv2.projectPoints(obj, rvec, tvec, matrix, distortion)[0]
            error = float(np.sqrt(np.mean(np.sum((observed - projected) ** 2, axis=2))))
            errors.append({'file': sample['file'], 'rms_px': error})
        reasons = []
        if rms > 1.0:
            reasons.append('Global RMS exceeds 1.0 px')
        if max(e['rms_px'] for e in errors) > 2.0:
            reasons.append('At least one view exceeds 2.0 px')
        if tilt < 10.0:
            reasons.append('Insufficient board tilt diversity (less than 10 degrees)')
        if not (0 < matrix[0, 2] < size[0] and 0 < matrix[1, 2] < size[1]):
            reasons.append('Principal point outside image')
        if not (0.1 * size[0] < matrix[0, 0] < 10 * size[0] and
                0.1 * size[1] < matrix[1, 1] < 10 * size[1]):
            reasons.append('Implausible focal length')
        projection = np.zeros((3, 4))
        projection[:, :3] = matrix
        def mat(value):
            return {'rows': int(value.shape[0]), 'cols': int(value.shape[1]),
                    'data': value.reshape(-1).tolist()}
        calibration = {'image_width': size[0], 'image_height': size[1],
                       'camera_name': 'smartcar_' + self.meta['camera'],
                       'distortion_model': 'plumb_bob', 'camera_matrix': mat(matrix),
                       'distortion_coefficients': mat(distortion.reshape(1, -1)),
                       'rectification_matrix': mat(np.eye(3)), 'projection_matrix': mat(projection)}
        yaml_text = yaml.safe_dump(calibration, default_flow_style=False)
        report = {'camera': self.meta['camera'], 'topic': self.meta['topic'], 'image_size': list(size),
                  'board': self.meta['board'], 'square_m': self.meta['square_m'],
                  'sample_count': len(samples), 'rms_px': float(rms), 'coverage': cov,
                  'normal_span_degrees': float(tilt), 'per_view': errors,
                  'automatic_checks_passed': not reasons, 'warnings': reasons,
                  'note': 'Training reprojection error only; visually verify new views before deployment.',
                  'yaml_sha256': hashlib.sha256(yaml_text.encode('utf-8')).hexdigest()}
        result_dir = os.path.join(self.directory, 'results', time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:8])
        os.makedirs(result_dir)
        atomic_write(os.path.join(result_dir, 'camera.yaml'), yaml_text)
        dump_json(os.path.join(result_dir, 'dataset.json'), self.meta)
        dump_json(os.path.join(result_dir, 'report.json'), report)
        report['result_directory'] = result_dir
        return report, matrix, distortion
