#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Local white-boundary follower. Pixel geometry, not metric localization."""
from __future__ import division
import math
import cv2
import numpy as np


class LaneDetector(object):
    def __init__(self, cfg):
        self.cfg = cfg

    def _boundary(self, segments, height):
        if len(segments) < 2:
            return None
        points = np.asarray([p for seg in segments for p in seg], dtype=float)
        best = None
        for seg in segments:
            (x1, y1), (x2, y2) = seg
            if abs(y2-y1) < 10:
                continue
            m = (x2-x1)/(y2-y1); b = x1-m*y1
            keep = np.abs(points[:, 0]-(m*points[:, 1]+b)) < 16
            if keep.sum() < 4:
                continue
            span = np.ptp(points[keep, 1])
            score = keep.sum() + span/8
            if best is None or score > best[0]:
                best = (score, keep)
        if best is None:
            return None
        pts = points[best[1]]
        for unused in range(3):
            coeff = np.polyfit(pts[:, 1], pts[:, 0], 1)
            errors = pts[:, 0]-np.polyval(coeff, pts[:, 1])
            keep = np.abs(errors) < 14
            if keep.all() or keep.sum() < 4:
                break
            pts = pts[keep]
        span = float(np.ptp(pts[:, 1]))
        rms = float(np.sqrt(np.mean((pts[:, 0]-np.polyval(coeff, pts[:, 1]))**2)))
        if span < height*.14 or rms > 12:
            return None
        return {'coeff': coeff.tolist(), 'span': span, 'rms': rms,
                'points': int(len(pts)), 'ymin': float(pts[:, 1].min()),
                'ymax': float(pts[:, 1].max())}

    def detect(self, frame):
        if frame is None or frame.ndim != 3:
            return {'valid': False, 'reason': 'invalid_frame'}, None
        img = cv2.resize(frame, (640, 480))
        h, w = img.shape[:2]; cfg = self.cfg
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = ((hsv[:, :, 1] <= cfg['white_s_max']) &
                (hsv[:, :, 2] >= cfg['white_v_min'])).astype(np.uint8)*255
        top = int(h*cfg['roi_top']); bottom = int(h*cfg['roi_bottom'])
        mask[:top] = 0; mask[bottom:] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        edges = cv2.Canny(mask, 50, 120)
        raw = cv2.HoughLinesP(edges, 1, np.pi/180, 18,
                              minLineLength=25, maxLineGap=24)
        sides = [[], []]; cx = cfg['image_center_x']; near = h*cfg['near_y']
        if raw is not None:
            for line in raw.reshape(-1, 4):
                x1, y1, x2, y2 = map(float, line)
                if abs(y2-y1) < 12:
                    continue
                m = (x2-x1)/(y2-y1); at_near = x1+m*(near-y1)
                if not .15 < abs(m) < 3.8:
                    continue
                seg = [(x1, y1), (x2, y2)]
                if m < 0 and -220 < at_near < cx-45:
                    sides[0].append(seg)
                elif m > 0 and cx+45 < at_near < w+220:
                    sides[1].append(seg)
        left = self._boundary(sides[0], h); right = self._boundary(sides[1], h)
        out = {'valid': False, 'reason': 'missing_boundary', 'left': left, 'right': right}
        if left is None or right is None:
            return out, self.draw(img, out)
        look = h*cfg['look_y']
        lx, rx = [float(np.polyval(line['coeff'], look)) for line in [left, right]]
        ln, rn = [float(np.polyval(line['coeff'], near)) for line in [left, right]]
        width = rx-lx; width_near = rn-ln
        if not (160 < width < 650 and width <= width_near < 1000 and lx < cx < rx):
            out['reason'] = 'implausible_lane_geometry'
            return out, self.draw(img, out)
        # Do not drive on long extrapolations of short white glare patches.
        if any(look < line['ymin']-20 or look > line['ymax']+20 for line in [left, right]):
            out['reason'] = 'lookahead_outside_observed_lines'
            return out, self.draw(img, out)
        center = (lx+rx)/2; error = (center-cx)/(width/2)
        raw_steer = float(np.clip(cfg['steering_gain']*error,
                                  -cfg['max_steering'], cfg['max_steering']))
        confidence = min(1., min(left['span'], right['span'])/120.)
        confidence *= max(0., 1.-max(left['rms'], right['rms'])/25.)
        out.update(valid=bool(confidence >= cfg['min_confidence']),
                   reason='ok' if confidence >= cfg['min_confidence'] else 'low_confidence',
                   confidence=float(confidence), error=float(error),
                   center_x=center, look_y=look, lane_width_px=width,
                   raw_steering=raw_steer)
        return out, self.draw(img, out)

    def draw(self, img, out):
        result = img.copy()
        for name, color in [('left', (0, 220, 0)), ('right', (0, 220, 0))]:
            line = out.get(name)
            if line:
                ys = np.linspace(line['ymin'], line['ymax'], 30)
                xy = np.array([[np.polyval(line['coeff'], y), y] for y in ys], np.int32)
                cv2.polylines(result, [xy], False, color, 2)
        if 'center_x' in out:
            y = int(out['look_y']); x = int(out['center_x'])
            cv2.circle(result, (x, y), 6, (0, 0, 255), -1)
            cv2.line(result, (int(self.cfg['image_center_x']), y-18),
                     (int(self.cfg['image_center_x']), y+18), (255, 200, 0), 2)
        text = '%s  steer=%+.1f  confidence=%.2f' % (
            out['reason'], out.get('raw_steering', 0), out.get('confidence', 0))
        cv2.putText(result, text, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, .48,
                    (0, 220, 0) if out['valid'] else (0, 0, 255), 1)
        return result


def check_scan(scan, now, cfg):
    if not scan or now-scan['received'] > cfg['scan_timeout']:
        return 'lidar_missing_or_stale'
    if not -.1 < now-scan['stamp'] < cfg['scan_stamp_timeout']:
        return 'lidar_timestamp_stale'
    valid = []; close = 0
    for i, r in enumerate(scan['ranges']):
        a = (scan['angle_min']+i*scan['angle_increment']+math.pi)%(2*math.pi)-math.pi
        if abs(a) > math.radians(cfg['obstacle_half_angle_deg']):
            continue
        if math.isnan(r) or math.isinf(r) or r < .15 or r > 25:
            continue
        valid.append(r)
        if r < cfg['stop_distance_m']:
            close += 1
    if len(valid) < 12:
        return 'insufficient_front_lidar_coverage'
    if close >= 3:
        return 'front_obstacle'
    return None


def permit_motion(now, deadline, frame_time, detection_time, scan, cfg):
    if now >= deadline:
        return 'duration_limit'
    if now-frame_time > cfg['camera_timeout']:
        return 'camera_stale'
    if now-detection_time > cfg['inference_timeout']:
        return 'lane_inference_stale'
    return check_scan(scan, now, cfg)
