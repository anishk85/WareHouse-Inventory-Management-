#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import termios
import tty
import time

# Joint limits
LOWER_LIMIT = 0.05   # meters
UPPER_LIMIT = 0.20   # meters
STEP = 0.01          # move 1 cm per key press

def get_key():
    """Read keyboard key without pressing Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class CameraTeleop(Node):
    def __init__(self):
        super().__init__('camera_keyboard_teleop')

        self.position = 0.05  # start at lower limit

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/qr_camera_controller/commands',
            10
        )

        self.get_logger().info("Keyboard teleop started.")
        self.print_instructions()
        self.send_position()

    def print_instructions(self):
        print("\n=== Camera Slider Keyboard Control ===")
        print("W -> Slide OUT (extend)")
        print("S -> Slide IN  (retract)")
        print("Q -> Quit")
        print("======================================\n")

    def send_position(self):
        msg = Float64MultiArray()
        msg.data = [self.position]
        self.pub.publish(msg)
        print(f"\rPosition: {self.position:.3f} m", end="")

    def update(self, key):
        if key.lower() == 'w':
            self.position += STEP
        elif key.lower() == 's':
            self.position -= STEP
        else:
            return

        # clamp to joint limits
        self.position = max(LOWER_LIMIT, min(UPPER_LIMIT, self.position))

        self.send_position()


def main(args=None):
    rclpy.init(args=args)
    node = CameraTeleop()

    try:
        while rclpy.ok():
            key = get_key()

            if key.lower() == 'q':
                print("\nExiting teleop...")
                break

            node.update(key)

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
