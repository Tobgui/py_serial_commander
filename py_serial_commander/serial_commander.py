#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class WheelVelPublisher(Node):
    def __init__(self):
        super().__init__('wheel_vel_publisher')
        self.subscription = self.create_subscription(
            Twist, '/diff_cont/cmd_vel_unstamped',
            self.cmd_vel_callback, 10)
        self.wheel_base = 0.3  # meters
        self.v = 0.0
        self.omega = 0.0
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info('Serial port opened.')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial open failed: {e}')
            self.ser = None
        self.timer = self.create_timer(0.1, self.send_serial_command)

    def cmd_vel_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def send_serial_command(self):
        v_r = (self.v + (self.wheel_base/2)*self.omega) * 100
        v_l = (self.v - (self.wheel_base/2)*self.omega) * 100
        serial_msg = f'L:{v_l:.2f},R:{v_r:.2f}\n'
        self.get_logger().info(f'Sending: {serial_msg.strip()}')
        if self.ser and self.ser.is_open:
            self.ser.write(serial_msg.encode())

def main(args=None):
    rclpy.init(args=args)
    node = WheelVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()