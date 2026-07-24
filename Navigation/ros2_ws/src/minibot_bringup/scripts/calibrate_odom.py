#!/usr/bin/env python3
"""
calibrate_odom.py -- fix wheel_radius and wheel_separation empirically.

EVERYTHING DOWNSTREAM IS BUILT ON THESE TWO NUMBERS. Do this once, properly.

    ros2 run minibot_bringup calibrate_odom.py --straight 2.0
    ros2 run minibot_bringup calibrate_odom.py --rotate 10

DO THE STRAIGHT-LINE TEST FIRST. Rotation error depends on wheel radius, so
calibrating rotation against an uncalibrated radius just moves the error around.

WHY IT LISTENS TO /diff_drive_controller/odom AND NOT /odom
    /odom is the EKF output -- wheel odometry ALREADY CORRECTED BY THE GYRO. If
    you calibrate against that, you are calibrating the wheels against the very
    filter that is compensating for them, and you will converge on nonsense.
    Calibrate against the RAW wheel odometry.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Calib(Node):
    def __init__(self, mode, target, speed):
        super().__init__('calibrate_odom')
        self.mode, self.target, self.speed = mode, target, speed
        self.start = None
        self.cur = None
        self.yaw_acc = 0.0
        self.last_yaw = None
        self.done = False

        # RAW wheel odometry. Not /odom (which is the gyro-corrected EKF output).
        self.create_subscription(Odometry, '/diff_drive_controller/odom',
                                 self._odom, 10)
        self.pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_timer(0.05, self._tick)

    def _odom(self, m):
        p = m.pose.pose
        if self.start is None:
            self.start = (p.position.x, p.position.y, yaw_of(p.orientation))
            self.last_yaw = self.start[2]
        self.cur = (p.position.x, p.position.y, yaw_of(p.orientation))

        y = self.cur[2]
        d = y - self.last_yaw
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        self.yaw_acc += d
        self.last_yaw = y

    def _cmd(self, vx, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = vx
        m.twist.angular.z = wz
        self.pub.publish(m)

    def _tick(self):
        if self.cur is None or self.done:
            return

        if self.mode == 'straight':
            d = math.dist(self.cur[:2], self.start[:2])
            if d >= self.target:
                self._cmd(0.0, 0.0)
                self.done = True
                self._report_straight(d)
            else:
                self._cmd(self.speed, 0.0)
                self.get_logger().info(f'{d:.3f} / {self.target:.3f} m',
                                       throttle_duration_sec=1.0)
        else:
            turns = abs(self.yaw_acc) / (2 * math.pi)
            if turns >= self.target:
                self._cmd(0.0, 0.0)
                self.done = True
                self._report_rotate(turns)
            else:
                self._cmd(0.0, self.speed)
                self.get_logger().info(f'{turns:.2f} / {self.target:.0f} turns',
                                       throttle_duration_sec=1.0)

    def _report_straight(self, reported):
        print(f'\n=== STRAIGHT LINE ===')
        print(f'  Robot REPORTS it travelled: {reported:.4f} m')
        print(f'\n  Now MEASURE the actual distance with a tape measure.')
        actual = float(input('  Actual distance (m): '))
        ratio = actual / reported
        print(f'\n  ratio = actual / reported = {ratio:.5f}')
        print(f'\n  FIX (choose ONE):')
        print(f'    a) robot.yaml -> geometry.wheel_radius *= {ratio:.5f}')
        print(f'       then: python3 tools/generate_config.py')
        print(f'    b) controllers.yaml -> set BOTH:')
        print(f'         left_wheel_radius_multiplier:  {ratio:.5f}')
        print(f'         right_wheel_radius_multiplier: {ratio:.5f}')
        print(f'\n  (a) is better -- it keeps robot.yaml the single source of truth.')

    def _report_rotate(self, reported):
        print(f'\n=== ROTATION ===')
        print(f'  Robot REPORTS it turned: {reported:.4f} revolutions')
        print(f'\n  Now MEASURE how far it ACTUALLY turned.')
        actual = float(input('  Actual revolutions (e.g. 9.75): '))
        mult = reported / actual
        print(f'\n  wheel_separation_multiplier = reported / actual = {mult:.5f}')
        print(f'\n  FIX (choose ONE):')
        print(f'    a) robot.yaml -> geometry.wheel_separation *= {mult:.5f}')
        print(f'       then: python3 tools/generate_config.py')
        print(f'    b) controllers.yaml -> wheel_separation_multiplier: {mult:.5f}')
        if mult > 1.0:
            print(f'\n  (Robot OVER-rotated: it turned more than it thought.')
            print(f'   That means wheel_separation is currently too SMALL.)')
        else:
            print(f'\n  (Robot UNDER-rotated: wheel_separation is too LARGE.)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--straight', type=float, metavar='METRES')
    ap.add_argument('--rotate', type=float, metavar='REVOLUTIONS')
    ap.add_argument('--speed', type=float, default=0.15,
                    help='m/s for straight, rad/s for rotate')
    a = ap.parse_args()

    if (a.straight is None) == (a.rotate is None):
        sys.exit('Pick exactly one: --straight METRES  or  --rotate REVOLUTIONS')

    mode = 'straight' if a.straight is not None else 'rotate'
    target = a.straight if a.straight is not None else a.rotate
    speed = a.speed if mode == 'straight' else max(a.speed, 0.6)

    rclpy.init()
    n = Calib(mode, target, speed)
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
