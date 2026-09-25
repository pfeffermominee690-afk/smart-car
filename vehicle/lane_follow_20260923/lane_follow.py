#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Bounded ROS lane-following trial; observe-only unless --execute is given."""
from __future__ import print_function, division
import argparse
import json
import os
import signal
import subprocess
import threading
import time
import traceback
import cv2
import rospy
import rosgraph
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from lane_vision import LaneDetector, permit_motion, check_scan

BASE = os.path.dirname(os.path.abspath(__file__))
SERIAL = '/dev/serial/by-id/usb-STMicroelectronics_ZDRB_USBCOM_368032723034-if00'
LIDAR = '/home/smartcar/newcar_ws/znxc/config/leishen_ws/devel/lib/ls01b_v2/ls01b_v2'


class Trial(object):
    def __init__(self, args):
        self.args = args
        self.cfg = json.load(open(os.path.join(BASE, 'config.json')))
        self.bridge = CvBridge()
        self.detector = LaneDetector(self.cfg)
        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0
        self.seq = 0
        self.scan = None
        self.scan_count = 0
        self.last_scan_stamp = 0
        self.detection_time = 0
        self.wanted = 0
        self.steering = 0
        self.active = False
        self.deadline = 0
        self.reason = None
        self.exit_requested = False
        self.worker_done = False
        self.children = []
        self.pub = None
        self.sent = 0
        self.first_motion = None
        self.out = args.output
        if not os.path.isdir(self.out):
            os.makedirs(self.out)
        self.events = open(os.path.join(self.out, 'frames.jsonl'), 'w')
        self.commands = open(os.path.join(self.out, 'commands.jsonl'), 'w')
        self.video = None
        self.worker = None

    def abort(self, reason):
        with self.lock:
            if self.reason is None:
                self.reason = reason
                print('STOP:', reason)
            self.active = False

    def image_cb(self, msg):
        try:
            now = time.time()
            stamp = msg.header.stamp.to_sec()
            if not -.1 < now-stamp < self.cfg['camera_timeout']:
                if self.active:
                    self.abort('camera_timestamp_stale')
                return
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.lock:
                self.frame = frame
                self.frame_time = stamp
                self.seq += 1
        except Exception:
            if self.active:
                self.abort('camera_decode_error')

    def scan_cb(self, msg):
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            self.scan = dict(received=time.time(), stamp=stamp,
                             ranges=list(msg.ranges), angle_min=msg.angle_min,
                             angle_increment=msg.angle_increment)
            if stamp > self.last_scan_stamp:
                self.scan_count += 1
                self.last_scan_stamp = stamp

    def spawn(self, name, command):
        stream = open(os.path.join(self.out, name+'.log'), 'w')
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   preexec_fn=os.setsid)
        self.children.append((process, stream))
        return process

    def port_free(self, path):
        with open(os.devnull, 'w') as devnull:
            code = subprocess.call(['fuser', path], stdout=devnull, stderr=devnull)
        if code == 0:
            raise RuntimeError('Port is in use: '+path)
        if not os.path.exists(path):
            raise RuntimeError('Port missing: '+path)

    def control_publishers(self):
        state = rosgraph.Master(rospy.get_name()).getSystemState()
        return [node for topic, nodes in state[0] if topic == '/ackermann_cmd'
                for node in nodes if node != rospy.get_name()]

    def send(self, speed, steering):
        message = AckermannDriveStamped()
        message.header.stamp = rospy.Time.now()
        message.drive.speed = speed
        message.drive.steering_angle = steering
        self.pub.publish(message)
        self.commands.write(json.dumps(dict(t=time.time(), speed=speed,
                                            steering=steering))+'\n')
        self.commands.flush()
        if speed:
            self.sent += 1
            if self.first_motion is None:
                self.first_motion = time.time()

    def control_loop(self):
        previous = time.time()
        next_graph_check = 0
        while not self.worker_done:
            now = time.time()
            if now >= next_graph_check:
                try:
                    if self.control_publishers():
                        self.abort('another_control_publisher')
                except Exception:
                    self.abort('ros_master_unreachable')
                next_graph_check = now+.5
            with self.lock:
                active = self.active
                problem = None
                if active:
                    problem = permit_motion(now, self.deadline, self.frame_time,
                                            self.detection_time, self.scan, self.cfg)
                    if self.pub.get_num_connections() < 1:
                        problem = 'base_subscriber_missing'
                    if os.path.exists(os.path.join(self.out, 'STOP')):
                        problem = 'stop_file'
                    if self.exit_requested:
                        problem = 'signal'
                if problem:
                    self.reason = self.reason or problem
                    self.active = active = False
                if active:
                    limit = self.cfg['steering_rate_per_s']*min(now-previous, .1)
                    self.steering += max(-limit, min(limit, self.wanted-self.steering))
                    speed = self.cfg['raw_speed']
                    steering = int(round(self.steering))
                else:
                    speed = steering = 0
            # Publishing never waits for camera decoding or lane detection.
            self.send(speed, steering)
            previous = now
            time.sleep(.05)

    def run(self):
        rospy.init_node('lane_follow_trial', anonymous=True, disable_signals=True)
        def signal_handler(signum, frame):
            self.exit_requested = True
            self.abort('signal')
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal_handler)
        rospy.Subscriber(self.cfg['image_topic'], Image, self.image_cb,
                         queue_size=1, buff_size=2**22)
        debug = rospy.Publisher('/lane_follow/debug', Image, queue_size=1)
        topics = dict(rospy.get_published_topics())
        scan_topic = self.cfg['scan_topic']
        if scan_topic not in topics:
            self.port_free('/dev/ttyUSB0')
            scan_topic = '/lane_follow/scan'
            self.spawn('lidar', [LIDAR, '__name:=lane_follow_lidar',
                'scan:='+scan_topic, '_serial_port:=/dev/ttyUSB0',
                '_baud_rate:=460800', '_angle_resolution:=0.25',
                '_robot_radius:=0.2', '_center_x:=-0.105', '_center_y:=0.0'])
        rospy.Subscriber(scan_topic, LaserScan, self.scan_cb, queue_size=1)
        if self.args.execute:
            if self.control_publishers():
                raise RuntimeError('Another controller is publishing /ackermann_cmd')
            self.port_free(SERIAL)
            self.spawn('base', ['roslaunch', 'base_controller', 'keyboard_drive.launch',
                               'serial_port:='+SERIAL])
            self.pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1)
            self.worker = threading.Thread(target=self.control_loop)
            self.worker.daemon = True
            self.worker.start()
        started = time.time()
        last_seq = -1
        stable = 0
        frames = 0
        valid_frames = 0
        armed = False
        observe_deadline = None
        last_status = 0
        while not self.exit_requested and not rospy.is_shutdown():
            if self.reason:
                break
            if self.worker and not self.worker.is_alive():
                self.abort('control_worker_failed')
                break
            now = time.time()
            if not armed and now-started > 20:
                self.abort('readiness_timeout')
                break
            with self.lock:
                seq = self.seq
                frame = self.frame
                frame_time = self.frame_time
                scan = self.scan
                scan_count = self.scan_count
            if frame is None or seq == last_seq:
                time.sleep(.01)
                continue
            last_seq = seq
            result, overlay = self.detector.detect(frame)
            now = time.time()
            fresh = -.1 < now-frame_time < self.cfg['camera_timeout']
            lidar_problem = check_scan(scan, now, self.cfg)
            usable = result['valid'] and fresh
            if usable:
                with self.lock:
                    self.detection_time = now
                    self.wanted = result['raw_steering']
            elif armed and self.args.execute:
                self.abort(result['reason'] if fresh else 'camera_stale')
            ready = usable and not lidar_problem and scan_count >= 8
            if self.pub is not None:
                ready = (ready and self.pub.get_num_connections() >= 1 and
                         rospy.get_param('/base_controller/serial_ready', False))
            stable = stable+1 if ready else 0
            if stable >= 10 and not armed:
                armed = True
                observe_deadline = now+self.args.duration
                with self.lock:
                    self.deadline = observe_deadline
                    self.active = self.args.execute and self.reason is None
                print('READY mode=%s duration=%.1fs' %
                      ('EXECUTE' if self.args.execute else 'OBSERVE', self.args.duration))
            if armed and now >= observe_deadline:
                self.abort('duration_limit')
            frames += 1
            valid_frames += int(result['valid'])
            result.update(t=now, camera_stamp=frame_time, seq=seq,
                          lidar_status=lidar_problem or 'ok', active=self.active)
            self.events.write(json.dumps(result)+'\n')
            self.events.flush()
            if overlay is not None:
                if self.video is None:
                    self.video = cv2.VideoWriter(os.path.join(self.out, 'overlay.avi'),
                        cv2.VideoWriter_fourcc(*'MJPG'), 10., (640, 480))
                self.video.write(overlay)
                if frames % 5 == 0:
                    cv2.imwrite(os.path.join(self.out, 'latest.jpg'), overlay)
                    debug.publish(self.bridge.cv2_to_imgmsg(overlay, 'bgr8'))
            if now-last_status > 1:
                print('frames=%d valid=%d stable=%d vision=%s lidar=%s active=%s' %
                      (frames, valid_frames, stable, result['reason'], lidar_problem, self.active))
                last_status = now
            time.sleep(.025)
        self.summary = dict(mode='execute' if self.args.execute else 'observe',
                            frames=frames, valid_frames=valid_frames, armed=armed)

    def cleanup(self):
        self.abort(self.reason or 'exit')
        self.worker_done = True
        if self.worker:
            self.worker.join(2)
        if self.pub:
            for unused in range(20):
                self.send(0, 0)
                time.sleep(.05)
        for process, stream in reversed(self.children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                until = time.time()+3
                while process.poll() is None and time.time() < until:
                    time.sleep(.05)
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
            stream.close()
        if self.video:
            self.video.release()
        summary = getattr(self, 'summary', {})
        summary.update(stop_reason=self.reason, nonzero_commands=self.sent,
                       first_motion=self.first_motion, finished=time.time(),
                       config=self.cfg)
        with open(os.path.join(self.out, 'summary.json'), 'w') as target:
            json.dump(summary, target, indent=2)
        self.events.close()
        self.commands.close()
        print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--duration', type=float, default=3.)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not 0 < args.duration <= 3:
        parser.error('This prototype is limited to 0 < duration <= 3 seconds')
    trial = Trial(args)
    try:
        trial.run()
    except Exception:
        traceback.print_exc()
        trial.reason = trial.reason or 'exception'
    finally:
        trial.cleanup()
