#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math
import time
import smbus2

MPU_ADDR    = 0x68
PWR_MGMT_1  = 0x6B
GYRO_ZOUT_H = 0x47
GYRO_SCALE  = 131.0

class MPU6050:
    def __init__(self, bus_num=1):
        self.bus = smbus2.SMBus(bus_num)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)
        time.sleep(0.1)
        self.bias = 0.0

    def _read_raw_z(self):
        high = self.bus.read_byte_data(MPU_ADDR, GYRO_ZOUT_H)
        low  = self.bus.read_byte_data(MPU_ADDR, GYRO_ZOUT_H + 1)
        val = (high << 8) | low
        if val > 32767:
            val -= 65536
        return val

    def read_z_rads(self):
        raw = self._read_raw_z()
        deg_per_s = raw / GYRO_SCALE
        return math.radians(deg_per_s) - self.bias

    def calibrate(self, samples=300):
        total = 0.0
        for _ in range(samples):
            raw = self._read_raw_z()
            total += math.radians(raw / GYRO_SCALE)
            time.sleep(0.003)
        self.bias = total / samples
        return self.bias


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.get_logger().info("Connecting to MPU6050...")
        try:
            self.imu = MPU6050()
            self.get_logger().info("MPU6050 connected.")
        except Exception as e:
            self.get_logger().error(f"MPU6050 FAILED: {e}")
            raise

        self.get_logger().info("Calibrating gyro — KEEP ROBOT PERFECTLY STILL...")
        bias = self.imu.calibrate()
        self.get_logger().info(f"Gyro bias = {bias:.6f} rad/s. Calibration done.")

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._vx = 0.0
        self._vth_cmd = 0.0
        self._last_time = self.get_clock().now()

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self._cmd_cb, 10)
        self.create_timer(0.02, self._publish_odom)

        self.get_logger().info("Odometry node ready — heading from GYRO.")

    def _cmd_cb(self, msg):
        self._vx = float(msg.linear.x)
        self._vth_cmd = float(msg.angular.z)

    def _publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        try:
            omega = self.imu.read_z_rads()
        except Exception:
            omega = self._vth_cmd

        if abs(omega) < 0.01:
            omega = 0.0

        self._theta += omega * dt
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        self._x += self._vx * math.cos(self._theta) * dt
        self._y += self._vx * math.sin(self._theta) * dt

        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)
        stamp = now.to_msg()

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0]  = 0.05
        odom.pose.covariance[7]  = 0.05
        odom.pose.covariance[35] = 0.02
        odom.twist.twist.linear.x  = self._vx
        odom.twist.twist.angular.z = omega
        self._odom_pub.publish(odom)


def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

main()
