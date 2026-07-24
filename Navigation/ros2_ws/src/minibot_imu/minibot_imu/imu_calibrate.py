#!/usr/bin/env python3
"""
imu_calibrate.py -- standalone IMU verification. NO ROS NEEDED.

    python3 imu_calibrate.py                 # full check
    python3 imu_calibrate.py --axis-check    # interactive axis-map wizard

RUN THIS BEFORE YOU TRUST THE EKF. An IMU whose z-axis is inverted will make
robot_localization confidently fuse a heading that turns the wrong way, and the
symptom (a map that tears itself apart during turns) looks nothing like the
cause.
"""

import argparse
import math
import time

import numpy as np
from smbus2 import SMBus, i2c_msg

PWR_MGMT_1, CONFIG, SMPLRT_DIV = 0x6B, 0x1A, 0x19
GYRO_CONFIG, ACCEL_CONFIG, WHO_AM_I, ACCEL_XOUT_H = 0x1B, 0x1C, 0x75, 0x3B
G, D2R, ALSB, GLSB = 9.80665, math.pi / 180.0, 16384.0, 131.0


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


class Mpu:
    def __init__(self, bus=1, addr=0x68):
        self.bus, self.addr = SMBus(bus), addr
        who = self.bus.read_byte_data(addr, WHO_AM_I)
        print(f'WHO_AM_I = 0x{who:02x}')
        self.bus.write_byte_data(addr, PWR_MGMT_1, 0x80); time.sleep(0.1)
        self.bus.write_byte_data(addr, PWR_MGMT_1, 0x01); time.sleep(0.05)
        self.bus.write_byte_data(addr, CONFIG, 0x03)
        self.bus.write_byte_data(addr, SMPLRT_DIV, 0x09)
        self.bus.write_byte_data(addr, GYRO_CONFIG, 0x00)
        self.bus.write_byte_data(addr, ACCEL_CONFIG, 0x00)
        time.sleep(0.05)

    def read(self):
        w = i2c_msg.write(self.addr, [ACCEL_XOUT_H])
        r = i2c_msg.read(self.addr, 14)
        self.bus.i2c_rdwr(w, r)
        d = list(r)
        a = np.array([s16(d[0], d[1]), s16(d[2], d[3]), s16(d[4], d[5])]) / ALSB * G
        g = np.array([s16(d[8], d[9]), s16(d[10], d[11]), s16(d[12], d[13])]) / GLSB * D2R
        return a, g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bus', type=int, default=1)
    ap.add_argument('--addr', type=lambda x: int(x, 0), default=0x68)
    ap.add_argument('--samples', type=int, default=500)
    ap.add_argument('--axis-check', action='store_true')
    a = ap.parse_args()

    m = Mpu(a.bus, a.addr)

    print(f'\nCollecting {a.samples} samples. HOLD THE ROBOT PERFECTLY STILL AND LEVEL.')
    accs, gyrs = [], []
    for _ in range(a.samples):
        av, gv = m.read()
        accs.append(av); gyrs.append(gv)
        time.sleep(0.002)
    accs, gyrs = np.array(accs), np.array(gyrs)

    gb, gs = gyrs.mean(axis=0), gyrs.std(axis=0)
    ab, asd = accs.mean(axis=0), accs.std(axis=0)

    print('\n--- GYRO ---')
    print('  bias  (rad/s): %+.5f %+.5f %+.5f' % tuple(gb))
    print('  noise (rad/s): %.5f %.5f %.5f' % tuple(gs))
    print('\n--- ACCEL ---')
    print('  mean  (m/s2) : %+.3f %+.3f %+.3f' % tuple(ab))
    print('  noise (m/s2) : %.3f %.3f %.3f' % tuple(asd))
    print('  |accel|      : %.3f  (expect ~9.81)' % np.linalg.norm(ab))

    print('\n--- VERDICT ---')
    ok = True
    if gs.max() > 0.05:
        print('  FAIL: gyro noise too high -- THE ROBOT MOVED during calibration.')
        ok = False
    if abs(np.linalg.norm(ab) - G) > 1.0:
        print('  FAIL: |accel| is not ~9.81. Wrong scale, or the robot is not still.')
        ok = False
    if ab[2] < -5.0:
        print('  ! accel z is NEGATIVE -> the board is UPSIDE DOWN.')
        print("    Set axis_map: ['x', '-y', '-z']")
        ok = False
    elif ab[2] < 5.0:
        print('  ! accel z is not ~+9.81 -> the board is NOT FLAT, or is rotated.')
        print('    Use --axis-check to work out the right axis_map.')
        ok = False
    print('  gyro_noise param  -> %.4f' % max(gs.max(), 0.005))
    print('  accel_noise param -> %.4f' % max(asd.max(), 0.05))
    if ok:
        print('  PASS -- accel z is up, gyro is quiet. Good to go.')

    if a.axis_check:
        print('\n=== AXIS CHECK ===')
        print('Rotate the robot COUNTER-CLOCKWISE (viewed from above) for 3 seconds.')
        input('Press Enter, then rotate...')
        t0, zs = time.time(), []
        while time.time() - t0 < 3.0:
            _, gv = m.read()
            zs.append(gv - gb)
            time.sleep(0.01)
        z = np.array(zs)[:, 2].mean()
        print(f'  mean gyro z during CCW rotation = {z:+.4f} rad/s')
        if z > 0.05:
            print('  CORRECT. Positive z on CCW rotation is the ROS convention.')
        elif z < -0.05:
            print('  INVERTED. Your z axis is backwards.')
            print("  Set axis_map to negate z, e.g. ['x', '-y', '-z'] or ['-x','y','-z'].")
        else:
            print('  No rotation detected -- did you actually turn it?')


if __name__ == '__main__':
    main()
