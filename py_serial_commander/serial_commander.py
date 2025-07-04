#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class WheelVelPublisher(Node):
    def __init__(self):
        super().__init__('wheel_vel_publisher')
        self.subscription = self.create_subscription(
            Twist, '/diff_cont/cmd_vel_unstamped',
            self.cmd_vel_callback, 10)

        self.wheel_base = 0.3  # meters
        self.v = 0.0
        self.omega = 0.0
        self.ser = None

        self.connect_serial()

        self.timer = self.create_timer(0.1, self.send_serial_command)

    def connect_serial(self):
        """Attempt to open the serial port safely."""
        try:
            time.sleep(2)  # Wait for Arduino reset
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info('✅ Serial port opened.')
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Serial open failed: {e}')
            self.ser = None

    def cmd_vel_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def send_serial_command(self):
        # Calculate wheel velocities
        v_r = (self.v + (self.wheel_base / 2) * self.omega) * 100
        v_l = (self.v - (self.wheel_base / 2) * self.omega) * 100

        # Clamp for safety (optional)
        v_l = max(min(v_l, 100), -100)
        v_r = max(min(v_r, 100), -100)

        serial_msg = f'L:{int(v_l)},R:{int(v_r)}\r\n'
        self.get_logger().info(f'Sending: {serial_msg.strip()}')

        # Ensure serial is connected
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial disconnected. Attempting to reconnect...")
            self.connect_serial()
            return

        try:
            self.ser.write(serial_msg.encode())

            # Read all available lines
            while self.ser.in_waiting:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    self.get_logger().info(f"Arduino: {line}")

        except serial.SerialException as e:
            self.get_logger().error(f"❌ Serial write/read error: {e}")
            self.ser.close()
            self.ser = None  # mark for reconnection

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.get_logger().info("Closing serial port")
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🔴 Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()