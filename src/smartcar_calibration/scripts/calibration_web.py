#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Loopback-only ROS image subscriber. Does not publish vehicle commands or change cameras."""
from __future__ import print_function

import argparse
import copy
import json
import os
import threading
import time
import uuid
try:
    from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
    from SocketServer import ThreadingMixIn
    from urlparse import urlparse
except ImportError:
    from http.server import HTTPServer, BaseHTTPRequestHandler
    from socketserver import ThreadingMixIn
    from urllib.parse import urlparse

import cv2
import rospkg
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from smartcar_calibration.core import Dataset, detect


class LocalServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class Application(object):
    def __init__(self, args):
        self.lock = threading.RLock()
        self.selected = 'front'
        self.token = uuid.uuid4().hex
        self.frames = {}
        self.snapshot = None
        self.jpeg = None
        self.results = {}
        self.busy = None
        self.message = 'Ready. Select camera, hold board still, then capture.'
        self.undistorted = False
        self.stop = threading.Event()
        self.bridge = CvBridge()
        self.board = (args.cols, args.rows)
        self.topics = {'front': args.front_topic, 'rear': args.rear_topic}
        self.datasets = {name: Dataset(args.session_dir, name, self.board, args.square_mm / 1000., topic)
                         for name, topic in self.topics.items()}
        self.subscribers = [rospy.Subscriber(topic, Image, self.receive, callback_args=name,
                                            queue_size=1, buff_size=4 * 1024 * 1024)
                            for name, topic in self.topics.items()]
        self.thread = threading.Thread(target=self.process)
        self.thread.daemon = True
        self.thread.start()

    def receive(self, msg, name):
        with self.lock:
            self.frames[name] = (msg, time.time())

    def age(self, msg, received):
        age = max(0., time.time() - received)
        if msg.header.stamp.to_sec() > 0:
            delta = (rospy.Time.now() - msg.header.stamp).to_sec()
            if delta < -2:
                return 999.0
            age = max(age, delta)
        return age

    def process(self):
        previous = None
        while not self.stop.is_set() and not rospy.is_shutdown():
            with self.lock:
                name = self.selected
                source = self.frames.get(name)
                result = self.results.get(name)
                undistorted = self.undistorted
            if source is not None:
                msg, received = source
                key = (name, received, undistorted, id(result))
                if key != previous:
                    try:
                        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
                        corners = detect(frame, self.board)
                        preview = frame.copy()
                        if undistorted and result is not None:
                            preview = cv2.undistort(frame, result[1], result[2])
                        elif corners is not None:
                            cv2.drawChessboardCorners(preview, self.board, corners, True)
                        label = name.upper() + (' UNDISTORTED' if undistorted and result else ' RAW')
                        cv2.putText(preview, label, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
                        ok, encoded = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if ok:
                            with self.lock:
                                if self.selected == name:
                                    self.snapshot = (name, frame, corners, msg, received)
                                    self.jpeg = encoded.tobytes()
                        previous = key
                    except Exception as exc:
                        rospy.logwarn_throttle(5, 'Calibration preview: %s' % exc)
            self.stop.wait(.25)

    def state(self):
        with self.lock:
            snapshot = self.snapshot
            age = self.age(snapshot[3], snapshot[4]) if snapshot else None
            return {'token': self.token, 'selected': self.selected, 'topics': self.topics,
                    'board': list(self.board), 'square_mm': self.datasets['front'].meta['square_m'] * 1000,
                    'datasets': {name: ds.status() for name, ds in self.datasets.items()},
                    'detected': snapshot is not None and snapshot[2] is not None and age <= 2,
                    'frame_age': age, 'busy': self.busy, 'message': self.message,
                    'undistorted': self.undistorted,
                    'results': {name: result[0] for name, result in self.results.items()}}

    def action(self, data):
        with self.lock:
            action = data.get('action')
            name = self.selected
            if action == 'select':
                if data.get('camera') not in self.datasets:
                    raise ValueError('Unknown camera')
                self.selected = data['camera']
                self.snapshot = self.jpeg = None
                self.undistorted = False
                return
            if data.get('camera') != name:
                raise ValueError('Camera selection changed; refresh before operating')
            if action == 'view':
                self.undistorted = not self.undistorted
                return
            if self.busy:
                raise ValueError('Calibration is running; wait until it finishes')
            dataset = self.datasets[name]
            if action == 'capture':
                if self.snapshot is None or self.snapshot[0] != name:
                    raise ValueError('No image from selected camera')
                _, frame, corners, msg, received = self.snapshot
                stamp = str(msg.header.stamp) if msg.header.stamp.to_sec() else str(received)
                dataset.add(frame, corners, self.age(msg, received), stamp)
                self.results.pop(name, None)
                self.undistorted = False
                self.message = '%s: captured %d samples' % (name, len(dataset.meta['samples']))
            elif action == 'undo':
                dataset.undo()
                self.results.pop(name, None)
                self.undistorted = False
                self.message = name + ': last sample excluded (PNG retained)'
            elif action == 'calibrate':
                self.busy = name
                self.message = name + ': calculating, please wait'
                worker = threading.Thread(target=self.solve, args=(name, copy.deepcopy(dataset)))
                worker.daemon = True
                worker.start()
            else:
                raise ValueError('Unknown action')

    def solve(self, name, dataset):
        try:
            result = dataset.calibrate()
            with self.lock:
                self.results[name] = result
                self.message = name + ': saved candidate; inspect report and undistorted new views'
        except Exception as exc:
            with self.lock:
                self.message = name + ': ' + str(exc)
            rospy.logwarn('Calibration failed: %s', exc)
        finally:
            with self.lock:
                self.busy = None


def make_handler(app, page):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            BaseHTTPRequestHandler.setup(self)
            self.connection.settimeout(10)

        def log_message(self, *args):
            pass

        def reply(self, status, kind, body):
            if not isinstance(body, bytes):
                body = body.encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            self.end_headers()
            self.wfile.write(body)

        def local_host(self):
            return urlparse('http://' + self.headers.get('Host', '')).hostname in ('localhost', '127.0.0.1')

        def do_GET(self):
            if not self.local_host():
                return self.reply(403, 'text/plain', 'Use a localhost SSH port forward')
            path = urlparse(self.path).path
            if path == '/':
                self.reply(200, 'text/html; charset=utf-8', page)
            elif path == '/state':
                self.reply(200, 'application/json', json.dumps(app.state()))
            elif path == '/frame.jpg':
                with app.lock:
                    jpeg = app.jpeg
                self.reply(200 if jpeg else 503, 'image/jpeg' if jpeg else 'text/plain', jpeg or 'No image')
            else:
                self.reply(404, 'text/plain', 'Not found')

        def do_POST(self):
            origin = self.headers.get('Origin')
            if (not self.local_host() or (origin and origin != 'http://' + self.headers.get('Host', ''))
                    or self.headers.get('Content-Type', '').split(';')[0] != 'application/json'
                    or self.path != '/action'):
                return self.reply(403, 'text/plain', 'Invalid local request')
            try:
                length = int(self.headers.get('Content-Length', 0))
                if not 0 < length < 4096:
                    raise ValueError('Invalid request size')
                data = json.loads(self.rfile.read(length).decode('utf-8'))
                if data.get('token') != app.token:
                    return self.reply(403, 'text/plain', 'Refresh page to obtain session token')
                app.action(data)
                self.reply(200, 'application/json', json.dumps(app.state()))
            except (ValueError, IOError, cv2.error) as exc:
                self.reply(400, 'application/json', json.dumps({'error': str(exc)}))
    return Handler


def main():
    parser = argparse.ArgumentParser(description='Independent front/rear pinhole calibration; ROS subscriber only')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--cols', type=int, default=8, help='Inner corners along long board edge')
    parser.add_argument('--rows', type=int, default=6, help='Inner corners along short board edge')
    parser.add_argument('--square-mm', type=float, default=25.0)
    parser.add_argument('--front-topic', default='/usb_cam_2/image')
    parser.add_argument('--rear-topic', default='/usb_cam_1/image')
    parser.add_argument('--session-dir', default=os.path.expanduser(
        '~/smartcar-data/calibration/session-' + time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:6]),
        help='Absolute external data directory; reuse this path to resume collected samples')
    args = parser.parse_args(rospy.myargv()[1:])
    args.session_dir = os.path.abspath(os.path.expanduser(args.session_dir))
    rospy.init_node('smartcar_calibration_web', anonymous=True)
    package = rospkg.RosPack().get_path('smartcar_calibration')
    with open(os.path.join(package, 'web', 'index.html'), 'rb') as stream:
        page = stream.read()
    # Reserve port before subscribing or creating a dataset.
    server = LocalServer(('127.0.0.1', args.port), make_handler(None, page))
    app = Application(args)
    server.RequestHandlerClass = make_handler(app, page)
    server.timeout = .5
    print('Calibration: http://127.0.0.1:%d' % args.port)
    print('Session directory: ' + args.session_dir)
    print('Resume with: --session-dir ' + args.session_dir)
    try:
        while not rospy.is_shutdown():
            server.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
        app.stop.set()
        server.server_close()


if __name__ == '__main__':
    main()
