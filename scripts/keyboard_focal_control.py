#!/usr/bin/env python3
"""Publish RViz focal point offsets from terminal arrow keys."""

import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker
from std_msgs.msg import String


class KeyboardFocalControl(Node):
    def __init__(self):
        super().__init__('keyboard_focal_control')
        self.step = self.declare_parameter('step', 1.0).value
        if not math.isfinite(self.step) or self.step <= 0:
            raise ValueError('step must be a finite positive number')
        self.publish_rate = self.declare_parameter('publish_rate', 50.0).value
        self.key_timeout = self.declare_parameter('key_timeout', 0.1).value
        for name, value in [('publish_rate', self.publish_rate),
                            ('key_timeout', self.key_timeout)]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be a finite positive number')
        self.repeat = ArrowRepeat(self.publish_rate, self.key_timeout)
        self.publisher = self.create_publisher(
            Vector3, '/rviz/focal_point_offset', 10)
        self.goal_publisher = self.create_publisher(
            PoseStamped, '/planning/mission_planning/goal', 10)
        self.reset_publisher = self.create_publisher(
            String, '/rviz/focal_point_reset', 10)
        self.focal_point = None
        self.focal_subscription = self.create_subscription(
            Marker, '/rviz/focal_point_marker', self.update_focal_point,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def update_focal_point(self, marker):
        if marker.ns != 'focal_crosshair' or marker.id != 0:
            return
        if marker.action != Marker.ADD or not marker.points:
            self.focal_point = None
            return
        if len(marker.points) == 6:
            # Compatibility with the original crosshair's world-space segments.
            self.focal_point = (
                marker.header.frame_id,
                sum(p.x for p in marker.points) / 6,
                sum(p.y for p in marker.points) / 6,
                sum(p.z for p in marker.points) / 6,
            )
        else:
            # Target reticles carry their exact center in the marker pose.
            p = marker.pose.position
            self.focal_point = (marker.header.frame_id, p.x, p.y, p.z)

    def publish_goal(self):
        if self.focal_point is None:
            self.get_logger().warning('No focal point received from RViz yet')
            return
        frame, x, y, z = self.focal_point
        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal)

    def reset_focal_point(self):
        self.repeat.stop()
        self.reset_publisher.publish(String(data='base_link'))

    def publish_arrow(self, key):
        offsets = {
            'A': (self.step, 0.0),
            'B': (-self.step, 0.0),
            'C': (0.0, -self.step),
            'D': (0.0, self.step),
        }
        x, y = offsets[key]
        self.publisher.publish(Vector3(x=x, y=y, z=0.0))


class ArrowRepeat:
    """Repeat only after consecutive terminal events establish a held key."""

    def __init__(self, rate, timeout):
        self.period = 1.0 / rate
        self.timeout = timeout
        self.stop()

    def stop(self):
        self.key = None
        self.last_event = float('-inf')
        self.next_publish = float('inf')
        self.repeating = False

    def press(self, key, now, publish):
        if key != self.key or now - self.last_event > self.timeout:
            self.repeating = False
            publish(key)
            self.next_publish = now + self.period
        else:
            self.repeating = True
        self.key = key
        self.last_event = now
        self.tick(now, publish)

    def tick(self, now, publish):
        if now - self.last_event >= self.timeout:
            self.stop()
        elif self.repeating and now >= self.next_publish:
            publish(self.key)
            # Do not catch up with a burst after a stalled event loop.
            self.next_publish = now + self.period


def read_keyboard(node):
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    pending = b''
    last_input = 0.0
    try:
        tty.setcbreak(fd)
        print('Arrow keys: up/down = +/-X, left/right = +/-Y. '
              f'Step: {node.step}, repeat: {node.publish_rate} Hz. Enter: publish goal. R: reset to base_link. Press q or Ctrl+C to quit.', flush=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            node.repeat.tick(time.monotonic(), node.publish_arrow)
            if not select.select([fd], [], [], min(0.01, 1.0 / node.publish_rate))[0]:
                if time.monotonic() - last_input > 0.2:
                    pending = b''
                continue
            data = os.read(fd, 32)
            if not data:
                break
            last_input = time.monotonic()
            for value in data:
                key = bytes([value])
                if key in (b'q', b'Q', b'\x03', b'\x04'):
                    return
                if key in (b'\r', b'\n'):
                    node.repeat.stop()
                    node.publish_goal()
                    pending = b''
                elif key in (b'r', b'R') and not pending:
                    node.reset_focal_point()
                    pending = b''
                elif key == b'\x1b':
                    pending = key
                elif pending == b'\x1b' and key in (b'[', b'O'):
                    pending += key
                elif pending in (b'\x1b[', b'\x1bO') and key in b'ABCD':
                    node.repeat.press(
                        key.decode('ascii'), time.monotonic(), node.publish_arrow)
                    pending = b''
                else:
                    pending = b''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def main(args=None):
    if not sys.stdin.isatty():
        print('Run this node in an interactive terminal.', file=sys.stderr)
        return 1
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardFocalControl()
        read_keyboard(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
