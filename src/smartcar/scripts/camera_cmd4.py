#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Single-node camera lane following with LiDAR obstacle avoidance."""
from __future__ import print_function

import argparse
import copy
from collections import Counter, deque
import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import Bool, String

from intersection_control import IntersectionController


class LanePID(object):
    def __init__(self, proportional_gain=2.5):
        self.kp = float(proportional_gain)
        self.last_time = time.time()
        self.last_error = 0.0
        self.output = 0.0

    def clear(self):
        self.last_time = time.time()
        self.last_error = 0.0
        self.output = 0.0

    def update(self, feedback_value):
        error = -float(feedback_value)
        self.last_time = time.time()
        self.last_error = error
        self.output = self.kp*error


class PreviousRightLaneTracker(object):
    """Right-boundary tracker taken from the previous camera_cmd3 logic."""

    def __init__(self):
        intrinsic = np.array([
            [444.92179141947196, 0.0, 337.9399577215935],
            [0.0, 443.59684870127415, 285.85005027454673],
            [0.0, 0.0, 1.0]], dtype=np.float64)
        distortion = np.array([
            -0.41251110946304526,
            0.3572865990594976,
            0.0010943780467337042,
            0.0007363828658040413,
            -0.2652600879338458], dtype=np.float64)
        self.src_points = np.float32([
            [274, 195], [69, 301], [569, 305], [386, 197]])
        destination = np.float32([
            [170, 0], [170, 480], [470, 480], [470, 0]])
        self.transform = cv2.getPerspectiveTransform(
            self.src_points, destination)
        self.inverse_transform = cv2.getPerspectiveTransform(
            destination, self.src_points)
        self.undistort_map1, self.undistort_map2 = (
            cv2.initUndistortRectifyMap(
                intrinsic, distortion, None, intrinsic,
                (640, 480), cv2.CV_16SC2))
        self.dilate_kernel = np.ones((15, 15), np.uint8)
        self.erode_kernel = np.ones((7, 7), np.uint8)

        self.lane_width_px = float(rospy.get_param(
            '~lane_width_px', 300.0))
        self.lane_width_tolerance_px = float(rospy.get_param(
            '~lane_width_tolerance_px', 60.0))
        self.target_center_correction_px = float(rospy.get_param(
            '~target_center_correction_px', 3.0))
        self.bird_fit_min_row = int(rospy.get_param(
            '~bird_fit_min_row', 240))
        self.control_near_min_row = int(rospy.get_param(
            '~control_near_min_row', 360))
        self.min_lane_pixels = int(rospy.get_param(
            '~min_lane_pixels', 200))
        self.right_search_inset_px = float(rospy.get_param(
            '~right_search_inset_px', 30.0))
        self.right_window_max_shift_px = float(rospy.get_param(
            '~right_window_max_shift_px', 60.0))
        self.right_fit_max_jump_px = float(rospy.get_param(
            '~right_fit_max_jump_px', 45.0))
        self.heading_gain = float(rospy.get_param(
            '~heading_gain', 0.01))
        self.heading_sample_row = int(rospy.get_param(
            '~heading_sample_row', 405))
        self.heading_max_command = float(rospy.get_param(
            '~heading_max_command', 8.0))
        self.curve_direction_threshold_px = float(rospy.get_param(
            '~curve_direction_threshold_px', 50.0))
        self.lane_loss_hold_seconds = float(rospy.get_param(
            '~lane_loss_hold_seconds', 1.0))
        self.lane_speed = float(rospy.get_param('~lane_speed', -37.0))
        self.max_steering = float(rospy.get_param(
            '~max_lane_steering', 25.0))
        self.pid = LanePID(rospy.get_param('~lane_kp', 2.5))
        self.previous_right_fit = None
        self.right_lane_lost_frames = 0
        self.right_lane_lost_since = None
        self.last_valid_speed = 0.0
        self.last_valid_steering = 0.0
        self.has_last_valid_command = False

    def reset(self):
        self.previous_right_fit = None
        self.right_lane_lost_frames = 0
        self.right_lane_lost_since = None
        self.last_valid_speed = 0.0
        self.last_valid_steering = 0.0
        self.has_last_valid_command = False
        self.pid.clear()

    @staticmethod
    def fit_x(fit, rows):
        return fit['a2']*rows**2 + fit['a1']*rows + fit['a0']

    def lane_pixels_valid(self, pixel_log, min_vertical_span=100):
        rows = np.asarray(pixel_log['x'])
        if len(rows) < self.min_lane_pixels:
            return False
        return (rows.max()-rows.min()) >= min_vertical_span

    @staticmethod
    def edge_candidates(histogram, peak_threshold,
                        relative_threshold=0.30):
        if histogram.size == 0:
            return []
        peak_value = float(np.max(histogram))
        if peak_value <= peak_threshold:
            return []
        threshold = max(
            float(peak_threshold), peak_value*relative_threshold)
        strong = np.flatnonzero(histogram >= threshold)
        if strong.size == 0:
            return []
        split_at = np.where(np.diff(strong) > 1)[0]+1
        groups = np.split(strong, split_at)
        candidates = []
        for group in groups:
            if group.size:
                candidates.append((
                    int(group[0]), float(np.max(histogram[group]))))
        return candidates

    def find_starter_near(self, image, expected_x, tolerance,
                          peak_threshold):
        x_start = max(0, int(round(expected_x-tolerance)))
        x_end = min(
            image.shape[1], int(round(expected_x+tolerance))+1)
        for height in (48, 96, image.shape[0]//2, image.shape[0]):
            crop = image[image.shape[0]-height:image.shape[0],
                         x_start:x_end]
            histogram = np.sum(crop, axis=0)
            candidates = self.edge_candidates(
                histogram, peak_threshold, 0.25)
            if not candidates:
                continue
            expected_local = expected_x-x_start
            edge_local, strength = min(
                candidates,
                key=lambda item: abs(item[0]-expected_local))
            return {'centroid': x_start+edge_local,
                    'intensity': strength}
        return {'centroid': int(round(expected_x)), 'intensity': 0}

    def run_right_window(self, image, centroid_starter,
                         peak_threshold=10):
        steps = 10
        height = int(round(float(image.shape[0])/steps))
        max_shift = self.right_window_max_shift_px
        window_width = max(90, 60, int(round(2.0*max_shift)))
        center = float(centroid_starter)
        last_center = None
        hotpixels = {'x': [], 'y': []}
        tracking_started = False
        missing_windows = 0
        for step in range(steps):
            y_end = image.shape[0]-step*height
            y_start = max(0, y_end-height)
            predicted_center = center
            if last_center is not None:
                predicted_center = center+(center-last_center)
            x_start = max(
                0, int(round(predicted_center-window_width/2.0)))
            x_end = min(image.shape[1], x_start+window_width)
            if x_end-x_start < window_width:
                x_start = max(0, x_end-window_width)
            histogram = np.sum(
                image[y_start:y_end, x_start:x_end], axis=0)
            candidates = self.edge_candidates(
                histogram, peak_threshold, 0.25)
            if not candidates:
                if tracking_started:
                    missing_windows += 1
                    if missing_windows >= 2:
                        break
                continue
            predicted_local = predicted_center-x_start
            edge_local, unused = min(
                candidates,
                key=lambda item: abs(item[0]-predicted_local))
            inner_x = x_start+int(edge_local)
            if abs(inner_x-predicted_center) > max_shift:
                if tracking_started:
                    missing_windows += 1
                    if missing_windows >= 2:
                        break
                continue
            if inner_x <= 6 or inner_x >= image.shape[1]-7:
                break
            support_start = max(x_start, inner_x-6)
            support_end = min(x_end, inner_x+7)
            hot_y, hot_x = np.nonzero(
                image[y_start:y_end, support_start:support_end])
            if hot_y.size == 0:
                continue
            hotpixels['x'].extend((hot_y+y_start).tolist())
            hotpixels['y'].extend((hot_x+support_start).tolist())
            last_center = center
            center = float(inner_x)
            tracking_started = True
            missing_windows = 0
        return hotpixels

    @staticmethod
    def polynomial_fit(data):
        a2, a1, a0 = np.polyfit(data['x'], data['y'], 2)
        return {'a0': a0, 'a1': a1, 'a2': a2}

    def make_bird_debug(self, warped, right_fit=None, status='',
                        target_points=None, draw_rows=None):
        bird = cv2.cvtColor(
            np.uint8(np.clip(warped, 0, 255)), cv2.COLOR_GRAY2BGR)
        if draw_rows is None:
            rows = np.linspace(0, bird.shape[0]-1, num=bird.shape[0])
        else:
            rows = np.asarray(draw_rows)
        if right_fit is not None:
            right_x = self.fit_x(right_fit, rows)
            right_path = np.column_stack(
                (right_x, rows)).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(bird, [right_path], False, (0, 255, 0), 3)
        if target_points is not None and len(target_points) >= 2:
            center_path = np.asarray(
                target_points).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(
                bird, [center_path], False, (0, 0, 255), 7)
        if status:
            cv2.putText(
                bird, status, (15, 32), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (0, 0, 255), 2)
        return bird

    def loss_result(self, reason, edges, warped):
        now = time.time()
        if self.right_lane_lost_since is None:
            self.right_lane_lost_since = now
        elapsed = now-self.right_lane_lost_since
        holding = (
            self.has_last_valid_command and
            elapsed <= self.lane_loss_hold_seconds)
        speed = self.last_valid_speed if holding else 0.0
        steering = self.last_valid_steering if holding else 0.0
        action = 'HOLD' if holding else 'STOP'
        bird = self.make_bird_debug(
            warped, status='RIGHT LANE LOST - '+action)
        overlay = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        cv2.putText(
            overlay, 'RIGHT LANE LOST - '+action, (15, 32),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        return ({
            'valid': holding,
            'reason': reason+'_hold' if holding else reason,
            'mode': 'right' if holding else 'none',
            'speed': float(speed),
            'steering': float(steering),
            'loss_elapsed': float(elapsed)}, overlay, bird)

    def detect(self, frame, stamp):
        corrected = cv2.remap(
            frame, self.undistort_map1, self.undistort_map2,
            cv2.INTER_LINEAR)
        gray = cv2.cvtColor(corrected, cv2.COLOR_RGB2GRAY)
        edges = cv2.Canny(gray, 200, 400)
        mask = np.zeros_like(edges)
        roi_top = max(
            0, int(np.floor(np.min(self.src_points[:, 1])))-5)
        vertices = np.array([[
            (0, roi_top), (0, edges.shape[0]-1),
            (edges.shape[1]-1, edges.shape[0]-1),
            (edges.shape[1]-1, roi_top)]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, 255)
        cleaned = cv2.bitwise_and(edges, mask)
        warped = cv2.warpPerspective(
            cleaned.astype(np.float32), self.transform,
            (cleaned.shape[1], cleaned.shape[0]),
            flags=cv2.INTER_LINEAR)
        warped = cv2.dilate(warped, self.dilate_kernel)
        warped = cv2.erode(warped, self.erode_kernel)
        tracking = warped.copy()
        fit_min = max(
            0, min(tracking.shape[0]-1, self.bird_fit_min_row))
        tracking[:fit_min, :] = 0

        calibrated_right_x = (
            tracking.shape[1]/2.0+self.lane_width_px/2.0)
        expected_right_x = (
            calibrated_right_x-self.right_search_inset_px)
        starter = self.find_starter_near(
            tracking, expected_right_x,
            self.lane_width_tolerance_px+40.0, 10)
        right_pixels = self.run_right_window(
            tracking, starter['centroid'], 10)
        if not self.lane_pixels_valid(right_pixels, 100):
            self.right_lane_lost_frames += 1
            if self.right_lane_lost_frames >= 3:
                self.previous_right_fit = None
            return self.loss_result(
                'right_lane_pixels_missing', edges, warped)

        candidate = self.polynomial_fit(right_pixels)
        detected_rows = np.asarray(right_pixels['x'], dtype=np.float64)
        detected_cols = np.asarray(right_pixels['y'], dtype=np.float64)
        residual = float(np.median(np.abs(
            detected_cols-self.fit_x(candidate, detected_rows))))
        fit_check_min = max(
            float(detected_rows.min()),
            min(float(self.control_near_min_row),
                float(detected_rows.max())-30.0))
        fit_check_rows = np.linspace(
            fit_check_min, float(detected_rows.max()), num=20)
        bottom_row = float(corrected.shape[0]-1)
        bottom_x = float(self.fit_x(candidate, bottom_row))
        fit_jump = 0.0
        if self.previous_right_fit is not None:
            fit_jump = float(np.max(np.abs(
                self.fit_x(candidate, fit_check_rows)-
                self.fit_x(self.previous_right_fit, fit_check_rows))))
        geometry_ok = (
            residual <= 12.0 and
            180.0 <= bottom_x <= warped.shape[1]-20.0)
        if not geometry_ok:
            self.right_lane_lost_frames += 1
            if self.right_lane_lost_frames >= 3:
                self.previous_right_fit = None
            return self.loss_result(
                'right_lane_geometry_rejected', edges, warped)

        temporal_jump = (
            self.previous_right_fit is not None and
            fit_jump > self.right_fit_max_jump_px)
        if temporal_jump:
            self.right_lane_lost_frames += 1
            if self.right_lane_lost_frames < 3:
                candidate = self.previous_right_fit.copy()
                detected_cols = self.fit_x(candidate, detected_rows)
            else:
                self.previous_right_fit = None
                return self.loss_result(
                    'right_lane_reacquire', edges, warped)
        else:
            self.previous_right_fit = candidate.copy()
            self.right_lane_lost_frames = 0
            self.right_lane_lost_since = None

        row_min = max(0.0, float(detected_rows.min()))
        row_max = min(
            float(corrected.shape[0]-1), float(detected_rows.max()))
        rows = np.linspace(
            row_min, row_max,
            num=max(2, int(row_max-row_min)+1))
        right_fit_x = self.fit_x(candidate, rows)
        near_mask = detected_rows >= float(self.control_near_min_row)
        near_valid = (
            np.count_nonzero(near_mask) >= 30 and
            np.ptp(detected_rows[near_mask]) >= 60.0)
        if near_valid:
            near_fit = np.polyfit(
                detected_rows[near_mask], detected_cols[near_mask], 1)
            control_min = max(
                row_min, float(self.control_near_min_row))
            control_rows = np.linspace(
                control_min, row_max,
                num=max(2, int(row_max-control_min)+1))
            control_right_x = np.polyval(near_fit, control_rows)
            right_bottom_x = float(np.polyval(near_fit, bottom_row))
        else:
            control_rows = rows
            control_right_x = right_fit_x
            right_bottom_x = float(self.fit_x(candidate, bottom_row))

        target_x = (
            control_right_x-self.lane_width_px/2.0+
            self.target_center_correction_px)
        target_valid = (
            (target_x >= 0.0) & (target_x < corrected.shape[1]) &
            (control_rows >= 0.0) &
            (control_rows < corrected.shape[0]))
        target_points = np.column_stack(
            (target_x[target_valid], control_rows[target_valid]))
        target_bottom_x = (
            right_bottom_x-self.lane_width_px/2.0+
            self.target_center_correction_px)
        metres_per_pixel = 0.6/self.lane_width_px
        offset = (
            corrected.shape[1]/2.0-target_bottom_x)*metres_per_pixel

        bird = self.make_bird_debug(
            warped, candidate, 'RIGHT LINE ONLY', target_points, rows)
        overlay = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        right_path = np.column_stack(
            (right_fit_x, rows)).astype(np.int32).reshape((-1, 1, 2))
        bird_overlay = np.zeros_like(overlay)
        cv2.polylines(
            bird_overlay, [right_path], False, (0, 255, 0), 3)
        if len(target_points) >= 2:
            center_path = target_points.astype(
                np.int32).reshape((-1, 1, 2))
            cv2.polylines(
                bird_overlay, [center_path], False, (0, 0, 255), 7)
        camera_overlay = cv2.warpPerspective(
            bird_overlay, self.inverse_transform,
            (corrected.shape[1], corrected.shape[0]),
            flags=cv2.INTER_NEAREST)
        path_mask = np.any(camera_overlay != 0, axis=2)
        overlay[path_mask] = camera_overlay[path_mask]

        self.pid.update(offset)
        lateral_command = -self.pid.output*40.0
        heading_row = max(
            row_min, min(float(self.heading_sample_row), row_max))
        right_ahead_x = float(self.fit_x(candidate, heading_row))
        path_delta_x = right_ahead_x-right_bottom_x
        heading_command = max(
            -self.heading_max_command,
            min(self.heading_max_command,
                -self.heading_gain*path_delta_x))
        steering_command = lateral_command+heading_command
        if path_delta_x < -self.curve_direction_threshold_px:
            steering_command = max(0.0, steering_command)
        elif path_delta_x > self.curve_direction_threshold_px:
            steering_command = min(0.0, steering_command)
        logical_steering = max(
            -self.max_steering,
            min(self.max_steering, steering_command))
        output_steering = -logical_steering
        self.last_valid_speed = self.lane_speed
        self.last_valid_steering = output_steering
        self.has_last_valid_command = True
        return ({
            'valid': True,
            'reason': 'tracking_previous_right_line',
            'mode': 'right',
            'speed': float(self.lane_speed),
            'steering': float(output_steering),
            'offset': float(offset),
            'lateral_command': float(lateral_command),
            'heading_command': float(heading_command),
            'path_delta_x': float(path_delta_x),
            'fit_residual': float(residual),
            'fit_jump': float(fit_jump)}, overlay, bird)


FOLLOW = 'FOLLOW'
AVOID_OUT = 'AVOID_OUT'
AVOID_ALIGN = 'AVOID_ALIGN'
PASS_OBSTACLE = 'PASS_OBSTACLE'
RETURN_IN = 'RETURN_IN'
RETURN_ALIGN = 'RETURN_ALIGN'
RECOVER = 'RECOVER'
RECOVER_BLEND = 'RECOVER_BLEND'
STOPPED = 'STOPPED'


class PassiveIntersectionPerception(object):
    """Observe blue stop lines and traffic signs without controlling the car."""

    SIGN_NAMES = ('straight', 'right', 'left', 'uturn', 'stop')
    TEMPLATE_FILES = ('go.png', 'tr.png', 'tl.png', 'tb.png', 'st.png')

    def __init__(self, tracker):
        self.undistort_map1 = tracker.undistort_map1
        self.undistort_map2 = tracker.undistort_map2
        self.blue_lower = np.array([100, 50, 50], dtype=np.uint8)
        self.blue_upper = np.array([124, 255, 255], dtype=np.uint8)
        self.blue_length_threshold = int(rospy.get_param(
            '~blue_length_threshold', 50))
        self.blue_confirm_frames = int(rospy.get_param(
            '~blue_confirm_frames', 3))
        self.blue_streak = 0

        self.sign_threshold = float(rospy.get_param(
            '~sign_match_threshold', 0.95))
        self.uturn_sign_threshold = float(rospy.get_param(
            '~uturn_sign_match_threshold', 0.90))
        rospy.loginfo(
            'Traffic-sign thresholds: default=%.3f uturn=%.3f',
            self.sign_threshold, self.uturn_sign_threshold)
        vote_window = max(1, int(rospy.get_param('~sign_vote_window', 8)))
        self.sign_confirm_votes = max(
            1, int(rospy.get_param('~sign_confirm_votes', 4)))
        self.sign_votes = deque(maxlen=vote_window)
        self.stable_sign = None
        self.templates = []
        template_dir = os.path.dirname(os.path.abspath(__file__))
        for filename in self.TEMPLATE_FILES:
            path = os.path.join(template_dir, filename)
            template = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if template is None:
                rospy.logwarn('Traffic-sign template missing: %s', path)
            self.templates.append(template)

    @staticmethod
    def draw_text(image, text, line, color):
        cv2.putText(
            image, text, (18, 32+line*30), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, color, 2)

    def reset_sign_votes(self):
        self.sign_votes.clear()
        self.stable_sign = None

    def detect_blue(self, corrected, overlay):
        small = cv2.resize(corrected, (96, 128))
        height, width = small.shape[:2]
        crop_y0 = int(2*height/5)
        crop_width = width-20
        roi = small[crop_y0:height, 0:crop_width]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        rows, columns = np.nonzero(mask)
        pixel_count = int(columns.size)
        length = int(np.unique(columns).size) if pixel_count else 0
        raw_trigger = length > self.blue_length_threshold
        if raw_trigger:
            self.blue_streak += 1
        else:
            self.blue_streak = 0
        confirmed = self.blue_streak >= self.blue_confirm_frames

        scale_x = overlay.shape[1]/float(width)
        scale_y = overlay.shape[0]/float(height)
        top = int(round(crop_y0*scale_y))
        right = int(round(crop_width*scale_x))
        enlarged = cv2.resize(
            mask, (right, overlay.shape[0]-top),
            interpolation=cv2.INTER_NEAREST)
        target = overlay[top:overlay.shape[0], 0:right]
        selected = enlarged > 0
        if np.any(selected):
            magenta = np.zeros_like(target)
            magenta[:, :] = (255, 0, 255)
            target[selected] = (
                0.35*target[selected]+0.65*magenta[selected]).astype(np.uint8)

        angle = 0.0
        mid_x = 0.0
        mid_y = 0.0
        if pixel_count > 2:
            mid_x = float(np.mean(columns))
            mid_y = float(np.mean(rows))
            slope, intercept = np.polyfit(columns, rows, 1)
            angle = float(np.arctan(slope)*180.0/np.pi)
            first = float(columns.min())
            last = float(columns.max())
            points = []
            for column in (first, last):
                row = float(slope*column+intercept)
                points.append((
                    int(round(column*scale_x)),
                    int(round((crop_y0+row)*scale_y))))
            cv2.line(overlay, points[0], points[1], (0, 255, 0), 4)
            cv2.circle(
                overlay,
                (int(round(mid_x*scale_x)),
                 int(round((crop_y0+mid_y)*scale_y))),
                7, (0, 255, 255), -1)
        cv2.rectangle(
            overlay, (0, top), (max(0, right-1), overlay.shape[0]-1),
            (255, 255, 0), 2)
        return {
            'raw': bool(raw_trigger),
            'confirmed': bool(confirmed),
            'streak': int(self.blue_streak),
            'length': length,
            'pixel_count': pixel_count,
            'mid_x': mid_x,
            'mid_y': mid_y,
            'angle_deg': angle}

    def detect_sign(self, corrected):
        gray = cv2.cvtColor(corrected, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 2, 2)
        edges = cv2.Canny(blurred, 150, 300)
        contour_result = cv2.findContours(
            edges.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_result[-2]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:30]
        candidates = []
        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            area = cv2.contourArea(contour)
            x, y, width, height = cv2.boundingRect(contour)
            if area <= 500 or height <= 0 or abs(height-width) >= 0.2*height:
                continue
            rectangle_error = abs((perimeter/4.0)**2-area)
            circle_error = abs(perimeter**2/(4.0*np.pi)-area)
            if rectangle_error < area*0.2 or circle_error < area*0.15:
                candidates.append((area, (x, y, width, height)))
        if not candidates:
            return None

        unused_area, box = max(candidates, key=lambda item: item[0])
        x, y, width, height = box
        crop = gray[y:y+height, x:x+width]
        if crop.size == 0:
            return None
        crop = cv2.resize(crop, (200, 200))
        scores = []
        for template in self.templates:
            if template is None:
                scores.append(-1.0)
                continue
            response = cv2.matchTemplate(
                crop, template, cv2.TM_CCORR_NORMED)
            scores.append(float(cv2.minMaxLoc(response)[1]))
        sign_class = int(np.argmax(scores))
        confidence = float(scores[sign_class])
        threshold = (
            self.uturn_sign_threshold
            if self.SIGN_NAMES[sign_class] == 'uturn'
            else self.sign_threshold)
        if confidence < threshold:
            return {'class_id': None, 'name': 'unknown',
                    'confidence': confidence, 'threshold': threshold,
                    'bbox': list(box)}
        return {'class_id': sign_class,
                'name': self.SIGN_NAMES[sign_class],
                'confidence': confidence, 'threshold': threshold,
                'bbox': list(box)}

    def update_sign_vote(self, detection):
        class_id = None if detection is None else detection.get('class_id')
        self.sign_votes.append(-1 if class_id is None else int(class_id))
        counts = Counter(value for value in self.sign_votes if value >= 0)
        if counts:
            winner, votes = counts.most_common(1)[0]
            if votes >= self.sign_confirm_votes:
                self.stable_sign = int(winner)
            elif len(self.sign_votes) == self.sign_votes.maxlen:
                self.stable_sign = None
        elif len(self.sign_votes) == self.sign_votes.maxlen:
            self.stable_sign = None
        return self.stable_sign

    def detect(self, frame, stamp):
        corrected = cv2.remap(
            frame, self.undistort_map1, self.undistort_map2,
            cv2.INTER_LINEAR)
        overlay = corrected.copy()
        blue = self.detect_blue(corrected, overlay)
        sign = self.detect_sign(corrected)
        stable_sign = self.update_sign_vote(sign)

        raw_name = 'none' if sign is None else sign['name']
        confidence = 0.0 if sign is None else sign['confidence']
        if sign is not None:
            x, y, width, height = sign['bbox']
            color = (0, 255, 0) if sign['class_id'] is not None else (0, 255, 255)
            cv2.rectangle(
                overlay, (x, y), (x+width, y+height), color, 3)
            cv2.putText(
                overlay, '%s %.3f' %
                (sign['name'].upper(), sign['confidence']),
                (x, max(25, y-8)), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, color, 2)
        stable_name = (
            'none' if stable_sign is None else self.SIGN_NAMES[stable_sign])
        self.draw_text(overlay, 'PASSIVE INTERSECTION PERCEPTION', 0,
                       (255, 255, 255))
        self.draw_text(
            overlay,
            'BLUE raw=%s confirmed=%s len=%d streak=%d' %
            (blue['raw'], blue['confirmed'], blue['length'], blue['streak']),
            1, (0, 0, 255) if blue['confirmed'] else (255, 255, 255))
        self.draw_text(
            overlay, 'SIGN raw=%s %.3f stable=%s' %
            (raw_name.upper(), confidence, stable_name.upper()),
            2, (0, 255, 0) if stable_sign is not None else (255, 255, 255))
        return ({
            'passive_only': True,
            'capture_stamp': float(stamp),
            'blue': blue,
            'sign_raw': sign,
            'sign_stable_id': stable_sign,
            'sign_stable_name': stable_name}, overlay)


class CameraLaneAvoidance(object):
    @staticmethod
    def bounded_param(name, default, minimum, maximum):
        value = float(rospy.get_param(name, default))
        bounded = max(float(minimum), min(float(maximum), value))
        if bounded != value:
            rospy.logwarn(
                'Clamped unsafe parameter %s from %.3f to %.3f',
                name, value, bounded)
        return bounded

    def __init__(self, ready_file=None):
        self.lock = threading.RLock()
        self.tracker_lock = threading.Lock()
        self.bridge = CvBridge()
        self.tracker = PreviousRightLaneTracker()

        self.camera_topic = rospy.get_param(
            '~camera_topic', '/usb_cam_2/image')
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.output_topic = rospy.get_param(
            '~output_topic', '/ackermann_cmd')
        self.perception_only = bool(rospy.get_param(
            '~perception_only', False))
        self.intersection_control_requested = bool(rospy.get_param(
            '~enable_intersection_control', False))
        self.intersection_control_enabled = (
            self.intersection_control_requested and
            not self.perception_only)
        self.intersection_enabled = (
            self.perception_only or self.intersection_control_requested or
            bool(rospy.get_param(
                '~enable_intersection_perception', False)))
        self.intersection_rate = max(
            0.2, float(rospy.get_param('~intersection_rate', 5.0)))
        self.intersection = (
            PassiveIntersectionPerception(self.tracker)
            if self.intersection_enabled else None)
        self.intersection_condition = threading.Condition()
        self.intersection_pending = None
        self.intersection_stopping = False
        self.intersection_last_queued = 0.0
        self.intersection_thread = None
        self.latest_intersection_result = None
        self.latest_intersection_time = 0.0
        self.intersection_vote_reset_requested = False
        self.tracker_reset_requested = False
        self.latest_intersection_command = (0.0, 0.0)
        self.intersection_result_timeout = self.bounded_param(
            '~intersection_result_timeout', 0.6, 0.2, 2.0)
        self.intersection_require_scan = bool(rospy.get_param(
            '~intersection_require_scan', False))
        # Keep forward traffic-sign maneuvers at the same speed as normal
        # lane following.  The lane speed is the single source of truth.
        intersection_forward_speed = self.tracker.lane_speed
        intersection_config = {
            'enabled': self.intersection_control_enabled,
            'approach_speed': intersection_forward_speed,
            'approach_timeout': self.bounded_param(
                '~intersection_approach_timeout', 2.0, 0.3, 4.0),
            'blue_clear_frames': max(1, min(10, int(rospy.get_param(
                '~blue_clear_frames', 2)))),
            'straight_speed': intersection_forward_speed,
            'straight_duration': self.bounded_param(
                '~straight_duration', 1.0, 0.1, 4.0),
            'right_speed': intersection_forward_speed,
            'right_angle': self.bounded_param(
                '~right_angle', 18.0, 3.0, 25.0),
            'right_duration': self.bounded_param(
                '~right_duration', 1.5, 0.1, 4.0),
            'left_speed': intersection_forward_speed,
            'left_angle': self.bounded_param(
                '~left_angle', -13.0, -25.0, -3.0),
            'left_duration': self.bounded_param(
                '~left_duration', 2.5, 0.1, 4.0),
            'uturn_forward_speed': intersection_forward_speed,
            'uturn_forward_angle': self.bounded_param(
                '~uturn_forward_angle', 20.0, 3.0, 25.0),
            'uturn_forward_duration': self.bounded_param(
                '~uturn_forward_duration', 1.0, 0.1, 4.0),
            'uturn_reverse_speed': self.bounded_param(
                '~uturn_reverse_speed', 10.0, 3.0, 20.0),
            'uturn_reverse_angle': self.bounded_param(
                '~uturn_reverse_angle', -20.0, -25.0, -3.0),
            'uturn_reverse_duration': self.bounded_param(
                '~uturn_reverse_duration', 0.8, 0.1, 4.0),
            'uturn_exit_speed': intersection_forward_speed,
            'uturn_exit_angle': self.bounded_param(
                '~uturn_exit_angle', 18.0, 3.0, 25.0),
            'uturn_exit_duration': self.bounded_param(
                '~uturn_exit_duration', 1.0, 0.1, 4.0),
            'reacquire_confirm_frames': max(
                1, min(20, int(rospy.get_param(
                    '~intersection_reacquire_confirm_frames', 3)))),
            'reacquire_timeout': self.bounded_param(
                '~intersection_reacquire_timeout', 3.0, 0.5, 8.0),
            'rearm_seconds': self.bounded_param(
                '~intersection_rearm_seconds', 2.0, 0.0, 10.0),
        }
        self.intersection_controller = IntersectionController(
            intersection_config)
        self.intersection_controller.state_started = time.time()

        # The calibrated lane controller uses positive steering for right and
        # negative steering for left on this car.
        self.avoid_left = bool(rospy.get_param('~avoid_left', True))
        self.avoid_speed = float(rospy.get_param('~avoid_speed', -15.0))
        self.turn_angle = float(rospy.get_param('~turn_angle', 12.0))
        self.out_duration = float(rospy.get_param('~out_duration', 0.55))
        self.align_duration = float(
            rospy.get_param('~align_duration', 0.55))
        self.pass_min_duration = float(
            rospy.get_param('~pass_min_duration', 0.50))
        self.pass_max_duration = float(
            rospy.get_param('~pass_max_duration', 2.50))
        self.return_duration = float(
            rospy.get_param('~return_duration', 0.55))
        self.return_align_duration = float(
            rospy.get_param('~return_align_duration', 0.55))
        self.recover_timeout = float(
            rospy.get_param('~recover_timeout', 3.0))
        self.blend_duration = float(
            rospy.get_param('~blend_duration', 0.50))

        # This car's 1440-point LS01B scan faces forward at index 720.
        self.front_index_ratio = float(
            rospy.get_param('~front_index_ratio', 0.5))
        self.obstacle_distance = float(
            rospy.get_param('~obstacle_distance', 0.90))
        self.clear_distance = float(
            rospy.get_param('~clear_distance', 1.05))
        self.corridor_half_width = float(
            rospy.get_param('~corridor_half_width', 0.32))
        self.obstacle_min_points = int(
            rospy.get_param('~obstacle_min_points', 6))
        self.obstacle_confirm_frames = int(
            rospy.get_param('~obstacle_confirm_frames', 2))
        self.clear_confirm_frames = int(
            rospy.get_param('~clear_confirm_frames', 3))
        self.side_distance = float(
            rospy.get_param('~side_distance', 0.65))
        self.side_min_points = int(
            rospy.get_param('~side_min_points', 8))
        self.side_clear_points = int(
            rospy.get_param('~side_clear_points', 3))

        self.image_timeout = float(
            rospy.get_param('~image_timeout', 0.35))
        self.scan_timeout = float(
            rospy.get_param('~scan_timeout', 0.40))
        self.recover_confirm_frames = int(
            rospy.get_param('~recover_confirm_frames', 8))
        self.recover_max_step = float(
            rospy.get_param('~recover_max_steering_step', 4.0))
        self.recover_max_delta = float(
            rospy.get_param('~recover_max_steering_delta', 12.0))

        self.state = FOLLOW
        self.state_started = time.time()
        self.last_image_time = 0.0
        self.last_scan_time = 0.0
        self.latest_lane_cmd = None
        self.latest_lane_result = None
        self.lane_valid = False
        self.preavoid_steering = 0.0
        self.recover_last_steering = None
        self.recover_frames = 0

        self.blocked_frames = 0
        self.side_clear_frames = 0
        self.side_seen = False
        self.front_blocked = False
        self.front_clear = True
        self.front_points = 0
        self.side_points = 0

        self.cmd_pub = None
        self.trajectory_pub = None
        self.bird_pub = None
        self.status_pub = None
        if not self.perception_only:
            self.cmd_pub = rospy.Publisher(
                self.output_topic, AckermannDriveStamped, queue_size=1)
            self.trajectory_pub = rospy.Publisher(
                '/camera_cmd4/trajectory/compressed',
                CompressedImage, queue_size=1)
            self.bird_pub = rospy.Publisher(
                '/camera_cmd4/bird/compressed',
                CompressedImage, queue_size=1)
            self.status_pub = rospy.Publisher(
                '/camera_cmd4/status', String, queue_size=1)
        self.intersection_image_pub = rospy.Publisher(
            '/camera_cmd4/intersection/compressed',
            CompressedImage, queue_size=1)
        self.intersection_status_pub = rospy.Publisher(
            '/camera_cmd4/intersection/status', String, queue_size=1)

        self.image_sub = rospy.Subscriber(
            self.camera_topic, Image,
            self.camera_callback, queue_size=1, buff_size=2**24)
        self.scan_sub = None
        self.reset_sub = None
        self.timer = None
        if not self.perception_only:
            self.scan_sub = rospy.Subscriber(
                self.scan_topic, LaserScan,
                self.scan_callback, queue_size=1)
            self.reset_sub = rospy.Subscriber(
                '/camera_cmd4/reset', Bool,
                self.reset_callback, queue_size=1)
            self.timer = rospy.Timer(
                rospy.Duration(0.05), self.control_timer)
        rospy.on_shutdown(self.shutdown)
        if self.intersection_enabled:
            self.intersection_thread = threading.Thread(
                target=self.intersection_worker)
            self.intersection_thread.daemon = True
            self.intersection_thread.start()
        if ready_file:
            with open(ready_file, 'w') as stream:
                stream.write('ready\n')
        rospy.loginfo(
            'camera_cmd4 ready: camera=%s scan=%s output=%s avoid_left=%s '
            'intersection_perception=%s intersection_control=%s '
            'perception_only=%s',
            self.camera_topic, self.scan_topic,
            self.output_topic, self.avoid_left,
            self.intersection_enabled, self.intersection_control_enabled,
            self.perception_only)
        if self.perception_only:
            rospy.logwarn(
                'PERCEPTION ONLY: no drive publisher, LiDAR subscriber, lane '
                'controller, or control timer was created')
            if self.intersection_control_requested:
                rospy.logwarn(
                    'Intersection control request ignored in perception-only '
                    'mode')
        elif self.intersection_enabled and not self.intersection_control_enabled:
            rospy.logwarn(
                'INTERSECTION PERCEPTION ONLY: blue lines and signs have no '
                'control effect')

    def transition(self, state, reason):
        if self.state == state:
            return
        rospy.logwarn(
            'Avoidance state %s -> %s: %s',
            self.state, state, reason)
        self.state = state
        self.state_started = time.time()
        if state == PASS_OBSTACLE:
            self.side_seen = False
            self.side_clear_frames = 0
        elif state == RECOVER:
            self.recover_frames = 0
            self.recover_last_steering = None
            self.lane_valid = False
            self.latest_lane_cmd = None
            # Discard any adjacent lane tracked while passing the obstacle.
            with self.tracker_lock:
                self.tracker.reset()

    def make_command(self, speed=0.0, steering=0.0):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.drive.speed = speed
        cmd.drive.steering_angle = steering
        return cmd

    def compressed(self, publisher, frame, header):
        if publisher.get_num_connections() == 0:
            return
        ok, encoded = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            message = CompressedImage()
            message.header = header
            message.format = 'jpeg'
            message.data = encoded.tobytes()
            publisher.publish(message)

    def queue_intersection_frame(self, frame, header):
        if not self.intersection_enabled:
            return
        now = time.time()
        if now-self.intersection_last_queued < 1.0/self.intersection_rate:
            return
        self.intersection_last_queued = now
        with self.intersection_condition:
            self.intersection_pending = (
                frame.copy(), copy.deepcopy(header))
            self.intersection_condition.notify()

    def intersection_worker(self):
        while not rospy.is_shutdown():
            with self.intersection_condition:
                while (self.intersection_pending is None and
                       not self.intersection_stopping and
                       not rospy.is_shutdown()):
                    self.intersection_condition.wait(0.25)
                if self.intersection_stopping or rospy.is_shutdown():
                    return
                frame, header = self.intersection_pending
                self.intersection_pending = None
            started = time.time()
            try:
                with self.lock:
                    reset_votes = self.intersection_vote_reset_requested
                    self.intersection_vote_reset_requested = False
                if reset_votes:
                    self.intersection.reset_sign_votes()
                result, overlay = self.intersection.detect(
                    frame, header.stamp.to_sec())
                result['processing_ms'] = (
                    time.time()-started)*1000.0
                with self.lock:
                    received = time.time()
                    self.latest_intersection_result = copy.deepcopy(result)
                    self.latest_intersection_time = received
                    intersection_state = self.intersection_controller.state
                    latched_sign = self.intersection_controller.latched_sign
                    blue_armed = self.intersection_controller.armed
                    state_elapsed = max(
                        0.0, received-
                        self.intersection_controller.state_started)
                    planned_speed, planned_steering = (
                        self.latest_intersection_command)
                result.update({
                    'passive_only': not self.intersection_control_enabled,
                    'intersection_state': intersection_state,
                    'latched_sign': latched_sign,
                    'control_enabled': self.intersection_control_enabled,
                    'blue_armed': blue_armed,
                    'state_elapsed': state_elapsed,
                    'planned_speed': planned_speed,
                    'planned_steering': planned_steering,
                })
                control_label = (
                    'CONTROL %s speed=%.0f steer=%+.1f' %
                    (intersection_state, planned_speed, planned_steering)
                    if self.intersection_control_enabled else
                    'NO CONTROL EFFECT')
                self.intersection.draw_text(
                    overlay, control_label, 3,
                    (0, 255, 255) if not self.intersection_control_enabled
                    else (255, 255, 255))
                self.intersection.draw_text(
                    overlay, 'LATCHED=%s ARMED=%s' %
                    (str(latched_sign).upper(), blue_armed),
                    4, (255, 255, 255))
                self.intersection_status_pub.publish(
                    String(data=json.dumps(result)))
                self.compressed(
                    self.intersection_image_pub, overlay, header)
                rospy.loginfo_throttle(
                    1.0,
                    'cmd4 intersection blue=%s len=%d sign=%s state=%s '
                    'time=%.0fms',
                    result['blue']['confirmed'],
                    result['blue']['length'],
                    result['sign_stable_name'],
                    intersection_state,
                    result['processing_ms'])
            except Exception as exc:
                rospy.logerr_throttle(
                    1.0, 'Passive intersection perception failed: %s', exc)

    def update_recovery(self, result):
        steering = float(result.get('steering', 0.0))
        valid = bool(result.get('valid', False))
        consistent = (
            abs(steering-self.preavoid_steering) <=
            self.recover_max_delta)
        stable = (
            self.recover_last_steering is None or
            abs(steering-self.recover_last_steering) <=
            self.recover_max_step)
        if valid and consistent and stable:
            self.recover_frames += 1
            self.recover_last_steering = steering
        else:
            self.recover_frames = 0
            self.recover_last_steering = None
        if self.recover_frames >= self.recover_confirm_frames:
            self.transition(
                RECOVER_BLEND,
                'right lane stable for %d frames' %
                self.recover_frames)

    def camera_callback(self, message):
        started = time.time()
        stamp = message.header.stamp.to_sec()
        if not -0.1 <= started-stamp <= self.image_timeout:
            with self.lock:
                self.lane_valid = False
            rospy.logwarn_throttle(
                1.0, 'camera_cmd4 rejected a stale camera frame')
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message, 'bgr8')
            if frame.shape != (480, 640, 3):
                raise ValueError(
                    'Calibration requires a 640x480 image')
            self.queue_intersection_frame(frame, message.header)
            if self.perception_only:
                return
            with self.lock:
                reset_tracker = self.tracker_reset_requested
                self.tracker_reset_requested = False
            with self.tracker_lock:
                if reset_tracker:
                    self.tracker.reset()
                result, overlay, bird = self.tracker.detect(
                    frame, stamp)
        except Exception as exc:
            with self.lock:
                self.lane_valid = False
            rospy.logerr_throttle(
                1.0, 'Curve detection failed: %s', exc)
            return

        result['capture_stamp'] = stamp
        result['processing_ms'] = (
            time.time()-started)*1000.0
        with self.lock:
            self.last_image_time = time.time()
            self.latest_lane_result = result
            self.lane_valid = bool(result.get('valid', False))
            if self.lane_valid:
                lane_cmd = AckermannDriveStamped()
                lane_cmd.header = message.header
                lane_cmd.drive.speed = float(result['speed'])
                lane_cmd.drive.steering_angle = float(
                    result['steering'])
                self.latest_lane_cmd = lane_cmd
            else:
                self.latest_lane_cmd = None

            if self.state == RECOVER:
                self.update_recovery(result)

            state_snapshot = self.state
            result['avoidance_state'] = state_snapshot
            result['front_blocked'] = self.front_blocked
            result['front_points'] = self.front_points
            result['side_points'] = self.side_points

        self.status_pub.publish(
            String(data=json.dumps(result)))
        self.compressed(
            self.trajectory_pub, overlay, message.header)
        self.compressed(
            self.bird_pub, bird, message.header)
        rospy.loginfo_throttle(
            0.5,
            'cmd4 lane=%s mode=%s state=%s speed=%.0f steer=%+.1f time=%.0fms',
            result['reason'], result['mode'], state_snapshot,
            result['speed'], result['steering'],
            result['processing_ms'])

    def relative_angle(self, index, count, increment):
        centre = self.front_index_ratio*float(count)
        return (float(index)-centre)*abs(increment)

    def scan_metrics(self, scan):
        count = len(scan.ranges)
        if count == 0:
            return 0, 0, 0

        front_points = 0
        clear_points = 0
        side_points = 0
        for index, distance in enumerate(scan.ranges):
            if ((math.isinf(distance) or math.isnan(distance)) or
                    distance <= max(0.03, scan.range_min) or
                    distance >= scan.range_max):
                continue
            angle = self.relative_angle(
                index, count, scan.angle_increment)
            if not -math.pi <= angle <= math.pi:
                continue
            forward = distance*math.cos(angle)
            lateral = distance*math.sin(angle)
            if (0.05 < forward < self.obstacle_distance and
                    abs(lateral) < self.corridor_half_width):
                front_points += 1
            if (0.05 < forward < self.clear_distance and
                    abs(lateral) < self.corridor_half_width):
                clear_points += 1

            angle_deg = angle*180.0/math.pi
            if self.avoid_left:
                obstacle_side = -120.0 < angle_deg < -20.0
            else:
                obstacle_side = 20.0 < angle_deg < 120.0
            if obstacle_side and distance < self.side_distance:
                side_points += 1
        return front_points, clear_points, side_points

    def scan_callback(self, scan):
        with self.lock:
            now = time.time()
            self.last_scan_time = now
            front_points, clear_points, side_points = (
                self.scan_metrics(scan))
            self.front_points = front_points
            self.side_points = side_points
            self.front_blocked = (
                front_points >= self.obstacle_min_points)
            self.front_clear = (
                clear_points <= self.side_clear_points)

            if self.state == FOLLOW:
                if self.front_blocked:
                    self.blocked_frames += 1
                else:
                    self.blocked_frames = 0
                if (not self.intersection_controller.active and
                        self.blocked_frames >=
                        self.obstacle_confirm_frames):
                    if self.latest_lane_cmd is not None:
                        self.preavoid_steering = (
                            self.latest_lane_cmd.drive.steering_angle)
                    else:
                        self.preavoid_steering = 0.0
                    self.transition(
                        AVOID_OUT,
                        'front obstacle confirmed (%d points)' %
                        front_points)
            elif self.state == PASS_OBSTACLE:
                if side_points >= self.side_min_points:
                    self.side_seen = True
                    self.side_clear_frames = 0
                elif (self.side_seen and
                      side_points <= self.side_clear_points):
                    self.side_clear_frames += 1
                else:
                    self.side_clear_frames = 0
                if (now-self.state_started >=
                        self.pass_min_duration and
                        self.front_clear and
                        self.side_seen and
                        self.side_clear_frames >=
                        self.clear_confirm_frames):
                    self.transition(
                        RETURN_IN,
                        'obstacle passed the side of the car')

    def avoidance_command(self):
        out_sign = -1.0 if self.avoid_left else 1.0
        if self.state == AVOID_OUT:
            return self.make_command(
                self.avoid_speed,
                out_sign*self.turn_angle)
        if self.state == AVOID_ALIGN:
            return self.make_command(
                self.avoid_speed,
                -out_sign*self.turn_angle)
        if self.state == PASS_OBSTACLE:
            return self.make_command(
                self.avoid_speed, 0.0)
        if self.state == RETURN_IN:
            return self.make_command(
                self.avoid_speed,
                -out_sign*self.turn_angle)
        if self.state == RETURN_ALIGN:
            return self.make_command(
                self.avoid_speed,
                out_sign*self.turn_angle)
        return self.make_command()

    def reset_callback(self, message):
        if not message.data:
            return
        with self.lock:
            if self.front_clear:
                self.blocked_frames = 0
                self.intersection_controller.reset(time.time())
                self.intersection_vote_reset_requested = True
                self.latest_intersection_result = None
                self.latest_intersection_time = 0.0
                self.latest_intersection_command = (0.0, 0.0)
                self.tracker_reset_requested = True
                self.transition(FOLLOW, 'manual reset')
            else:
                rospy.logwarn(
                    'Reset refused: forward corridor occupied')

    def update_intersection_control(self, now):
        latest = self.latest_intersection_result
        fresh = (
            latest is not None and
            now-self.latest_intersection_time <=
            self.intersection_result_timeout)
        blue = {} if latest is None else latest.get('blue', {})
        perception = {
            'fresh': bool(fresh),
            'sample_id': (
                self.latest_intersection_time if fresh else None),
            'blue_confirmed': bool(blue.get('confirmed', False)),
            'blue_raw': bool(blue.get('raw', False)),
            'stable_sign': (
                'none' if latest is None else
                latest.get('sign_stable_name', 'none')),
        }
        image_fresh = now-self.last_image_time <= self.image_timeout
        lane_valid = bool(self.lane_valid and image_fresh)
        lane_steering = (
            self.latest_lane_cmd.drive.steering_angle
            if self.latest_lane_cmd is not None else 0.0)
        confirmed_obstacle = (
            self.blocked_frames >= self.obstacle_confirm_frames)
        safety_fault = None
        if (self.intersection_controller.active and
                self.intersection_require_scan and
                (self.last_scan_time <= 0.0 or
                 now-self.last_scan_time > self.scan_timeout)):
            safety_fault = 'laser scan timeout during intersection'

        previous_state = self.intersection_controller.state
        output = self.intersection_controller.step(
            now, perception, self.state, lane_valid, lane_steering,
            confirmed_obstacle, safety_fault=safety_fault,
            lane_sample_id=self.last_image_time)
        if output['reset_sign_votes']:
            self.intersection_vote_reset_requested = True
            self.latest_intersection_result = None
            self.latest_intersection_time = 0.0
        if output['reset_tracker']:
            self.tracker_reset_requested = True
            self.lane_valid = False
            self.latest_lane_cmd = None
        if output['fault'] is not None and self.state != STOPPED:
            self.transition(STOPPED, output['fault'])
        if output['state'] != previous_state:
            rospy.logwarn(
                'Intersection state %s -> %s sign=%s fault=%s',
                previous_state, output['state'],
                output['latched_sign'], output['fault'])
        self.latest_intersection_command = (
            output['command']
            if output['command'] is not None else (0.0, 0.0))
        return output

    def control_timer(self, unused):
        with self.lock:
            now = time.time()
            elapsed = now-self.state_started

            intersection_output = self.update_intersection_control(now)
            intersection_active = intersection_output['active']

            if (not intersection_active and
                    self.state == AVOID_OUT and
                    elapsed >= self.out_duration):
                self.transition(
                    AVOID_ALIGN,
                    'outward steering duration complete')
                elapsed = 0.0
            elif (not intersection_active and
                  self.state == AVOID_ALIGN and
                  elapsed >= self.align_duration):
                self.transition(
                    PASS_OBSTACLE,
                    'vehicle aligned beside obstacle')
                elapsed = 0.0
            elif (not intersection_active and
                  self.state == PASS_OBSTACLE and
                  elapsed >= self.pass_max_duration):
                if self.front_clear:
                    self.transition(
                        RETURN_IN,
                        'pass timeout with front corridor clear')
                else:
                    self.transition(
                        STOPPED,
                        'pass timeout while obstacle remains ahead')
                elapsed = 0.0
            elif (not intersection_active and
                  self.state == RETURN_IN and
                  elapsed >= self.return_duration):
                self.transition(
                    RETURN_ALIGN,
                    'return steering duration complete')
                elapsed = 0.0
            elif (not intersection_active and
                  self.state == RETURN_ALIGN and
                  elapsed >= self.return_align_duration):
                self.transition(
                    RECOVER,
                    'returned to estimated original corridor')
                elapsed = 0.0
            elif (not intersection_active and
                  self.state == RECOVER and
                  elapsed >= self.recover_timeout):
                self.transition(
                    STOPPED,
                    'original lane was not confirmed')
                elapsed = 0.0
            elif (not intersection_active and
                  self.state == RECOVER_BLEND and
                  elapsed >= self.blend_duration):
                self.transition(
                    FOLLOW,
                    'lane controller takeover complete')
                elapsed = 0.0

            avoiding = self.state in (
                AVOID_OUT, AVOID_ALIGN, PASS_OBSTACLE,
                RETURN_IN, RETURN_ALIGN)
            if (not intersection_active and avoiding and
                    now-self.last_scan_time > self.scan_timeout):
                self.transition(
                    STOPPED, 'laser scan timeout')

            if intersection_active:
                speed, steering = (
                    intersection_output['command']
                    if intersection_output['command'] is not None
                    else (0.0, 0.0))
                command = self.make_command(speed, steering)
            elif self.state == FOLLOW:
                image_fresh = (
                    now-self.last_image_time <=
                    self.image_timeout)
                # Stop while the next scan confirms an obstacle candidate.
                if self.blocked_frames > 0:
                    command = self.make_command()
                elif (image_fresh and self.lane_valid and
                        self.latest_lane_cmd is not None):
                    command = copy.deepcopy(
                        self.latest_lane_cmd)
                else:
                    command = self.make_command()
            elif self.state == RECOVER_BLEND:
                image_fresh = (
                    now-self.last_image_time <=
                    self.image_timeout)
                if (not image_fresh or not self.lane_valid or
                        self.latest_lane_cmd is None):
                    self.transition(
                        STOPPED,
                        'lane lost during takeover')
                    command = self.make_command()
                else:
                    ratio = min(
                        1.0,
                        elapsed/max(0.01, self.blend_duration))
                    command = self.make_command(
                        self.avoid_speed*(1.0-ratio) +
                        self.latest_lane_cmd.drive.speed*ratio,
                        self.latest_lane_cmd.drive.steering_angle*
                        ratio)
            elif avoiding:
                command = self.avoidance_command()
            else:
                command = self.make_command()

            self.cmd_pub.publish(command)
            rospy.loginfo_throttle(
                0.5,
                'cmd4 state=%s obstacle=%d side=%d lane=%s speed=%.0f steer=%+.1f',
                '%s/%s' % (
                    self.state, self.intersection_controller.state),
                self.front_points,
                self.side_points, self.lane_valid,
                command.drive.speed,
                command.drive.steering_angle)

    def shutdown(self):
        with self.intersection_condition:
            self.intersection_stopping = True
            self.intersection_pending = None
            self.intersection_condition.notifyAll()
        if self.cmd_pub is not None:
            with self.lock:
                self.cmd_pub.publish(self.make_command())
        if (self.intersection_thread is not None and
                self.intersection_thread.is_alive()):
            self.intersection_thread.join(1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ready-file')
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('camera_cmd4')
    CameraLaneAvoidance(args.ready_file)
    rospy.spin()


if __name__ == '__main__':
    main()
