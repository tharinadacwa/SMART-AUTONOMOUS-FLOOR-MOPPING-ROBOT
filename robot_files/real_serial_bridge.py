import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

WHEEL_BASE_M = 0.450
BAUD_RATE = 115200
UART_PORT = '/dev/ttyAMA0'

class RealSerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        try:
            self.ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
            time.sleep(1)
            self.get_logger().info(f"✅ Connected to STM32 on {UART_PORT}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial failed: {e}")
            self.ser = None
        # Listen to BOTH topics — teleop uses /cmd_vel, Nav2 uses /cmd_vel_nav
        self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cb, 10)
        self.get_logger().info("✅ Listening to /cmd_vel AND /cmd_vel_nav")

    def cb(self, msg):
        vl = msg.linear.x - msg.angular.z * (WHEEL_BASE_M / 2.0)
        vr = msg.linear.x + msg.angular.z * (WHEEL_BASE_M / 2.0)
        cmd = f"V {vl:.4f} {vr:.4f}\n"
        if self.ser:
            self.ser.write(cmd.encode())
        print(f"→ STM32: '{cmd.strip()}'")

def main():
    rclpy.init()
    rclpy.spin(RealSerialBridge())

main()
