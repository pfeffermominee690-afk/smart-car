"""Filtered pixel-error PD control; invalid detections never become driveable."""
from __future__ import division
import math


class SteeringController(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.history = []
        self.previous = None
        self.stamp = None
        self.derivative = 0.

    def update(self, result, stamp):
        if not result.get('valid'):
            self.reset()
            return None
        error = result['error']
        if math.isnan(error) or math.isinf(error):
            self.reset()
            return None
        dt = stamp-self.stamp if self.stamp is not None else 0.
        if self.stamp is not None and (dt <= 0 or dt > .3):
            self.reset()
            return None
        self.history = (self.history+[error])[-3:]
        ordered = sorted(self.history)
        # Median suppresses a one-frame jump without accepting an invalid frame.
        filtered = ordered[len(ordered)//2] if len(ordered) == 3 else error
        if self.previous is not None:
            rate = (filtered-self.previous)/dt
            alpha = dt/(self.cfg['derivative_tau']+dt)
            self.derivative += alpha*(rate-self.derivative)
        p = self.cfg['steering_gain']*filtered
        d = max(-self.cfg['derivative_limit'], min(self.cfg['derivative_limit'],
                self.cfg['steering_kd']*self.derivative))
        command = max(-self.cfg['max_steering'], min(self.cfg['max_steering'], p+d))
        self.previous = filtered
        self.stamp = stamp
        return dict(raw_steering=command, filtered_error=filtered, p_term=p,
                    d_term=d, error_rate=self.derivative)
