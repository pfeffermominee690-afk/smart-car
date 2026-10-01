#!/usr/bin/env python
from __future__ import print_function


IDLE = 'INTERSECTION_IDLE'
WAIT_SIGN = 'INTERSECTION_WAIT_SIGN'
APPROACH = 'INTERSECTION_APPROACH'
STRAIGHT = 'INTERSECTION_STRAIGHT'
RIGHT = 'INTERSECTION_RIGHT'
LEFT = 'INTERSECTION_LEFT'
UTURN_FORWARD = 'INTERSECTION_UTURN_FORWARD'
UTURN_REVERSE = 'INTERSECTION_UTURN_REVERSE'
UTURN_EXIT = 'INTERSECTION_UTURN_EXIT'
REACQUIRE = 'INTERSECTION_REACQUIRE'
STOPPED = 'INTERSECTION_STOPPED'


class IntersectionController(object):
    DIRECTION_STATES = {
        'straight': STRAIGHT,
        'right': RIGHT,
        'left': LEFT,
        'uturn': UTURN_FORWARD,
    }

    def __init__(self, config):
        self.config = dict(config)
        self.state = IDLE
        self.state_started = 0.0
        self.latched_sign = None
        self.held_lane_steering = 0.0
        self.blue_clear_count = 0
        self.reacquire_count = 0
        self.armed = True
        self.rearm_at = 0.0
        self.fault = None
        self.last_perception_sample = None
        self.last_lane_sample = None

    @property
    def active(self):
        return self.state != IDLE

    def transition(self, state, now):
        self.state = state
        self.state_started = float(now)

    def reset(self, now=0.0):
        self.state = IDLE
        self.state_started = float(now)
        self.latched_sign = None
        self.held_lane_steering = 0.0
        self.blue_clear_count = 0
        self.reacquire_count = 0
        self.armed = False
        self.rearm_at = float(now)+float(self.config['rearm_seconds'])
        self.fault = None
        self.last_perception_sample = None
        self.last_lane_sample = None

    def stop_with_fault(self, now, reason):
        self.fault = str(reason)
        self.transition(STOPPED, now)

    def command_for_state(self):
        commands = {
            STRAIGHT: (
                self.config['straight_speed'], 0.0),
            RIGHT: (
                self.config['right_speed'], self.config['right_angle']),
            LEFT: (
                self.config['left_speed'], self.config['left_angle']),
            UTURN_FORWARD: (
                self.config['uturn_forward_speed'],
                self.config['uturn_forward_angle']),
            UTURN_REVERSE: (
                self.config['uturn_reverse_speed'],
                self.config['uturn_reverse_angle']),
            UTURN_EXIT: (
                self.config['uturn_exit_speed'],
                self.config['uturn_exit_angle']),
        }
        return commands.get(self.state, (0.0, 0.0))

    def state_duration(self):
        durations = {
            STRAIGHT: self.config['straight_duration'],
            RIGHT: self.config['right_duration'],
            LEFT: self.config['left_duration'],
            UTURN_FORWARD: self.config['uturn_forward_duration'],
            UTURN_REVERSE: self.config['uturn_reverse_duration'],
            UTURN_EXIT: self.config['uturn_exit_duration'],
        }
        return float(durations[self.state])

    def result(self, command=None, reset_sign_votes=False,
               reset_tracker=False, completed=False):
        return {
            'state': self.state,
            'active': self.active,
            'command': command,
            'reset_sign_votes': bool(reset_sign_votes),
            'reset_tracker': bool(reset_tracker),
            'completed': bool(completed),
            'latched_sign': self.latched_sign,
            'fault': self.fault,
            'armed': self.armed,
        }

    def update_rearm(self, now, perception, new_perception):
        if self.armed or not new_perception:
            return
        if perception.get('blue_raw', False):
            self.blue_clear_count = 0
            return
        self.blue_clear_count += 1
        if (self.blue_clear_count >= int(self.config['blue_clear_frames']) and
                now >= self.rearm_at):
            self.armed = True
            self.blue_clear_count = 0

    def begin_reacquire(self, now):
        self.reacquire_count = 0
        self.last_lane_sample = None
        self.transition(REACQUIRE, now)
        return self.result(command=(0.0, 0.0), reset_tracker=True)

    def step(self, now, perception, avoidance_state, lane_valid,
             lane_steering, front_blocked, safety_fault=None,
             lane_sample_id=None):
        now = float(now)
        perception_sample = perception.get('sample_id')
        new_perception = (
            perception_sample is not None and
            perception_sample != self.last_perception_sample)
        if new_perception:
            self.last_perception_sample = perception_sample
        if self.state == IDLE:
            self.update_rearm(now, perception, new_perception)
            if (not self.config.get('enabled', False) or not self.armed or
                    avoidance_state != 'FOLLOW' or
                    not perception.get('fresh', False) or
                    not perception.get('blue_confirmed', False)):
                return self.result()
            self.held_lane_steering = float(lane_steering)
            self.latched_sign = None
            self.blue_clear_count = 0
            self.fault = None
            self.transition(WAIT_SIGN, now)
            return self.result(
                command=(0.0, 0.0), reset_sign_votes=True)

        if front_blocked:
            self.stop_with_fault(now, 'front obstacle during intersection')
            return self.result(command=(0.0, 0.0))

        if safety_fault:
            self.stop_with_fault(now, safety_fault)
            return self.result(command=(0.0, 0.0))

        if self.state == STOPPED:
            return self.result(command=(0.0, 0.0))

        if (self.state != WAIT_SIGN and
                not perception.get('fresh', False)):
            self.stop_with_fault(now, 'intersection perception timeout')
            return self.result(command=(0.0, 0.0))

        if self.state == WAIT_SIGN:
            sign = perception.get('stable_sign', 'none')
            if perception.get('fresh', False) and sign in self.DIRECTION_STATES:
                self.latched_sign = sign
                self.blue_clear_count = 0
                self.transition(APPROACH, now)
            return self.result(command=(0.0, 0.0))

        if self.state == APPROACH:
            if not perception.get('fresh', False):
                self.stop_with_fault(now, 'intersection perception timeout')
                return self.result(command=(0.0, 0.0))
            if now-self.state_started >= float(
                    self.config['approach_timeout']):
                self.stop_with_fault(now, 'blue line approach timeout')
                return self.result(command=(0.0, 0.0))
            if new_perception:
                if perception.get('blue_raw', False):
                    self.blue_clear_count = 0
                else:
                    self.blue_clear_count += 1
            if self.blue_clear_count >= int(self.config['blue_clear_frames']):
                self.transition(self.DIRECTION_STATES[self.latched_sign], now)
                return self.result(command=self.command_for_state())
            return self.result(command=(
                float(self.config['approach_speed']),
                self.held_lane_steering))

        if self.state in (
                STRAIGHT, RIGHT, LEFT, UTURN_FORWARD,
                UTURN_REVERSE, UTURN_EXIT):
            if now-self.state_started < self.state_duration():
                return self.result(command=self.command_for_state())
            if self.state in (STRAIGHT, RIGHT, LEFT, UTURN_EXIT):
                return self.begin_reacquire(now)
            if self.state == UTURN_FORWARD:
                self.transition(UTURN_REVERSE, now)
            else:
                self.transition(UTURN_EXIT, now)
            return self.result(command=self.command_for_state())

        if self.state == REACQUIRE:
            new_lane_sample = (
                lane_sample_id is not None and
                lane_sample_id != self.last_lane_sample)
            if new_lane_sample:
                self.last_lane_sample = lane_sample_id
                if lane_valid:
                    self.reacquire_count += 1
                else:
                    self.reacquire_count = 0
            if self.reacquire_count >= int(
                    self.config['reacquire_confirm_frames']):
                self.state = IDLE
                self.state_started = now
                self.latched_sign = None
                self.armed = False
                self.blue_clear_count = 0
                self.rearm_at = now+float(self.config['rearm_seconds'])
                return self.result(completed=True)
            if now-self.state_started >= float(
                    self.config['reacquire_timeout']):
                self.stop_with_fault(now, 'right lane reacquire timeout')
                return self.result(command=(0.0, 0.0))
            return self.result(command=(0.0, 0.0))

        self.stop_with_fault(now, 'invalid intersection state')
        return self.result(command=(0.0, 0.0))
