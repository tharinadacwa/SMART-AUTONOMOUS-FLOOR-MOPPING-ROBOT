#!/usr/bin/env python3
"""
mpu6050_node.py -- MPU6050 driver on the Pi 5's I2C-1.

WHO CALLS IT      minibot_bringup/robot.launch.py
WHAT IT PUBLISHES /imu/data_raw   (sensor_msgs/Imu)
                  /diagnostics    (diagnostic_msgs/DiagnosticArray)
WHO CONSUMES IT   imu_filter_madgwick -> /imu/data -> robot_localization EKF
WHEN IT RUNS      continuously, at imu_rate_hz (100 Hz)

WHY THIS SENSOR IS NOT OPTIONAL ON THIS ROBOT
    There are no encoders. "Wheel odometry" here is really "the steps we asked
    for". That is excellent in a straight line -- you counted every pulse -- and
    MEDIOCRE in yaw: a 2 mm error in your wheel_separation measurement, or a hair
    of slip on any of the ~19 in-place turns per room, dumps straight into
    heading. Heading error is what destroys maps and makes coverage lanes fan out.

    The gyro measures yaw RATE directly. It does not care about wheel slip, wheel
    radius, or wheel separation. Fusing it fixes precisely the axis the steppers
    are worst at. That is the whole argument.

ORIENTATION IS NOT ESTIMATED HERE
    We set orientation_covariance[0] = -1, the ROS convention for "I have no
    orientation, ignore this field". Madgwick computes it downstream. Doing it in
    one place stops two filters fighting over the same quantity.

WIRING (Pi 5 40-pin header)
    VCC -> pin 1  (3V3)   <-- NOT 5V. The breakout's regulator is often bypassed,
    GND -> pin 6           and 5V on SDA/SCL will eventually kill the Pi's pins.
    SDA -> pin 3  (GPIO2)
    SCL -> pin 5  (GPIO3)
    AD0 -> GND => address 0x68   (3V3 => 0x69)
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, Temperature
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from smbus2 import SMBus, i2c_msg

# --- MPU6050 registers ------------------------------------------------------
PWR_MGMT_1   = 0x6B
PWR_MGMT_2   = 0x6C
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
WHO_AM_I     = 0x75
ACCEL_XOUT_H = 0x3B

GRAVITY  = 9.80665
DEG2RAD  = math.pi / 180.0
ACC_LSB  = 16384.0   # +/- 2 g
GYRO_LSB = 131.0     # +/- 250 deg/s
TEMP_LSB = 340.0


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


class Mpu6050Node(Node):

    def __init__(self):
        super().__init__('mpu6050_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('calib_samples', 500)

        # Chip axes -> ROS body frame (x forward, y left, z up).
        # Chip flat, silkscreen up, +X arrow forward -> ['x','y','z']
        # Chip upside down                           -> ['x','-y','-z']
        # Chip rotated 90 deg CCW                    -> ['-y','x','z']
        # FIX IT HERE, not in the EKF. The EKF cannot distinguish a mounting
        # error from a real motion, and it will happily fuse your mistake.
        self.declare_parameter('axis_map', ['x', 'y', 'z'])

        self.declare_parameter('gyro_noise', 0.02)    # rad/s std dev
        self.declare_parameter('accel_noise', 0.20)   # m/s^2 std dev

        # If the gyro bias drifts more than this between the startup calibration
        # and a later stationary period, warn: the chip is probably heating up
        # or the mount is loose.
        self.declare_parameter('bias_warn_rad_s', 0.02)

        g = self.get_parameter
        self.bus_num = int(g('i2c_bus').value)
        self.addr    = int(g('i2c_address').value)
        self.frame   = g('frame_id').value
        self.rate    = float(g('rate_hz').value)
        self.axis    = [str(a).lower() for a in g('axis_map').value]
        self.bias_warn = float(g('bias_warn_rad_s').value)

        gn = float(g('gyro_noise').value)
        an = float(g('accel_noise').value)
        self.gyro_cov  = gn * gn
        self.accel_cov = an * an

        self.bus = SMBus(self.bus_num)
        self._configure()

        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self._calibrate(int(g('calib_samples').value))

        # Health tracking for diagnostics
        self.read_errors = 0
        self.total_reads = 0
        self.last_temp = 0.0
        self.last_gyro = np.zeros(3)
        self.last_accel = np.zeros(3)
        self.stationary_since = None

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Imu, 'imu/data_raw', qos)
        self.pub_temp = self.create_publisher(Temperature, 'imu/temperature', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_service(Trigger, 'imu/recalibrate', self._srv_recal)

        self.create_timer(1.0 / self.rate, self._tick)
        self.create_timer(1.0, self._diagnostics)

        self.get_logger().info(
            f'MPU6050 up: bus {self.bus_num} @ 0x{self.addr:02x}, {self.rate:.0f} Hz, '
            f'frame "{self.frame}", axis_map {self.axis}')

    # ---------------- device -------------------------------------------------

    def _configure(self):
        who = self.bus.read_byte_data(self.addr, WHO_AM_I)
        # 0x68 = MPU6050. 0x70/0x71/0x73 = MPU6500/9250 family, whose accel+gyro
        # register map is identical, so they work here unchanged.
        if who not in (0x68, 0x69, 0x70, 0x71, 0x73, 0x98):
            self.get_logger().warn(
                f'Unexpected WHO_AM_I 0x{who:02x}. Is this really an MPU6050?')

        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x80)   # reset
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x01)   # clock = PLL/X-gyro
        time.sleep(0.05)
        self.bus.write_byte_data(self.addr, PWR_MGMT_2, 0x00)   # all axes on
        # DLPF 3: gyro 42 Hz / accel 44 Hz bandwidth, 1 kHz internal rate.
        # Filtering here (in the chip) is far better than filtering later: it
        # removes the motor-vibration energy BEFORE it aliases into the sample.
        self.bus.write_byte_data(self.addr, CONFIG, 0x03)
        self.bus.write_byte_data(self.addr, SMPLRT_DIV, 0x09)   # 1 kHz/10 = 100 Hz
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)  # +/- 250 dps
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0x00) # +/- 2 g
        self.bus.write_byte_data(self.addr, INT_ENABLE, 0x00)
        time.sleep(0.05)

    def _read_raw(self):
        """Burst-read 14 bytes so accel, temp and gyro come from ONE sample."""
        w = i2c_msg.write(self.addr, [ACCEL_XOUT_H])
        r = i2c_msg.read(self.addr, 14)
        self.bus.i2c_rdwr(w, r)
        d = list(r)

        ax = s16(d[0],  d[1])  / ACC_LSB  * GRAVITY
        ay = s16(d[2],  d[3])  / ACC_LSB  * GRAVITY
        az = s16(d[4],  d[5])  / ACC_LSB  * GRAVITY
        tp = s16(d[6],  d[7])  / TEMP_LSB + 36.53
        gx = s16(d[8],  d[9])  / GYRO_LSB * DEG2RAD
        gy = s16(d[10], d[11]) / GYRO_LSB * DEG2RAD
        gz = s16(d[12], d[13]) / GYRO_LSB * DEG2RAD
        return np.array([ax, ay, az]), np.array([gx, gy, gz]), tp

    def _calibrate(self, n):
        self.get_logger().info(
            f'Calibrating gyro bias over {n} samples. KEEP THE ROBOT PERFECTLY STILL.')
        gs = []
        for _ in range(n):
            try:
                _, g, _ = self._read_raw()
                gs.append(g)
            except OSError:
                pass
            time.sleep(0.002)

        if len(gs) < n // 2:
            self.get_logger().error(
                'Too many I2C failures during calibration. Check wiring; '
                '`i2cdetect -y 1` must show 68.')
            return

        arr = np.array(gs)
        self.gyro_bias = arr.mean(axis=0)
        std = arr.std(axis=0)

        self.get_logger().info(
            'Gyro bias (rad/s): x=%+.5f y=%+.5f z=%+.5f  |  noise sigma: '
            '%.5f %.5f %.5f' % (*self.gyro_bias, *std))

        # If the robot moved during calibration the bias is garbage and every
        # heading estimate downstream inherits it. Catch it here.
        if std.max() > 0.05:
            self.get_logger().error(
                f'Gyro noise during calibration was {std.max():.4f} rad/s -- far too '
                'high. THE ROBOT MOVED. The bias is wrong and your heading will '
                'drift. Restart the node and hold it still.')

    def _srv_recal(self, req, resp):
        self._calibrate(int(self.get_parameter('calib_samples').value))
        resp.success = True
        resp.message = 'Gyro bias: %+.5f %+.5f %+.5f' % tuple(self.gyro_bias)
        return resp

    # ---------------- axis remap --------------------------------------------

    def _remap(self, v):
        out = []
        for key in self.axis:
            neg = key.startswith('-')
            idx = {'x': 0, 'y': 1, 'z': 2}[key[-1]]
            out.append(-v[idx] if neg else v[idx])
        return np.array(out)

    # ---------------- publish -----------------------------------------------

    def _tick(self):
        self.total_reads += 1
        try:
            a, g, t = self._read_raw()
        except OSError as e:
            self.read_errors += 1
            self.get_logger().warn(f'I2C read failed: {e}', throttle_duration_sec=2.0)
            return

        g = g - self.gyro_bias
        a = self._remap(a)
        g = self._remap(g)

        self.last_gyro = g
        self.last_accel = a
        self.last_temp = t

        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame

        # -1 = "no orientation here". Madgwick supplies it downstream.
        m.orientation_covariance[0] = -1.0

        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = g
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = a

        for i in (0, 4, 8):
            m.angular_velocity_covariance[i] = self.gyro_cov
            m.linear_acceleration_covariance[i] = self.accel_cov

        self.pub.publish(m)

        tm = Temperature()
        tm.header = m.header
        tm.temperature = float(t)
        tm.variance = 0.0
        self.pub_temp.publish(tm)

    # ---------------- diagnostics -------------------------------------------

    def _diagnostics(self):
        st = DiagnosticStatus()
        st.name = 'minibot_imu: MPU6050'
        st.hardware_id = f'i2c-{self.bus_num}:0x{self.addr:02x}'
        st.level = DiagnosticStatus.OK
        st.message = 'OK'

        err_rate = self.read_errors / max(1, self.total_reads)
        gnorm = float(np.linalg.norm(self.last_gyro))
        anorm = float(np.linalg.norm(self.last_accel))

        if err_rate > 0.05:
            st.level = DiagnosticStatus.ERROR
            st.message = f'I2C error rate {err_rate*100:.1f}% -- check wiring'
        elif abs(anorm - GRAVITY) > 2.0:
            # If |accel| is not ~9.81 while stationary, the scale factor or the
            # axis map is wrong, OR the robot is being shaken.
            st.level = DiagnosticStatus.WARN
            st.message = (f'|accel| = {anorm:.2f} m/s^2, expected ~9.81. '
                          'Bad axis_map, or the robot is moving.')
        elif self.last_temp > 60.0:
            st.level = DiagnosticStatus.WARN
            st.message = f'IMU running hot ({self.last_temp:.0f} C) -- bias will drift'

        st.values = [
            KeyValue(key='gyro_bias_x', value=f'{self.gyro_bias[0]:+.5f}'),
            KeyValue(key='gyro_bias_y', value=f'{self.gyro_bias[1]:+.5f}'),
            KeyValue(key='gyro_bias_z', value=f'{self.gyro_bias[2]:+.5f}'),
            KeyValue(key='gyro_norm_rad_s', value=f'{gnorm:.4f}'),
            KeyValue(key='accel_norm_m_s2', value=f'{anorm:.3f}'),
            KeyValue(key='temperature_C', value=f'{self.last_temp:.1f}'),
            KeyValue(key='i2c_errors', value=str(self.read_errors)),
            KeyValue(key='i2c_error_rate', value=f'{err_rate*100:.2f}%'),
        ]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_diag.publish(arr)

    def destroy_node(self):
        try:
            self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Mpu6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
