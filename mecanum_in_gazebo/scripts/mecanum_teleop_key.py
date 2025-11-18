#!/usr/bin/env python3
"""
Keyboard teleoperation node for mecanum robot
Controls: WASD for movement, QE for rotation, X to stop
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

max_speed = 50.0  # Maximum wheel speed in rad/s


class MecanumTeleopKey(Node):
    def __init__(self):
        super().__init__("mecanum_teleop_key")

        # Publisher - publishing directly to controller commands
        self.publisher = self.create_publisher(
            Float64MultiArray, "/mecanum_drive_controller/commands", 10
        )

        # Movement parameters (wheel velocities in rad/s)
        self.wheel_speed = 20.0  # Base wheel speed
        self.speed_increment = 2.0

        # Mecanum drive kinematics
        # wheel order: [front_left, front_right, back_left, back_right]
        self.movement_bindings = {
            "w": [1, 1, 1, 1],  # Forward
            "s": [-1, -1, -1, -1],  # Backward
            "a": [-1, 1, 1, -1],  # Strafe left
            "d": [1, -1, -1, 1],  # Strafe right
            "q": [-1, 1, -1, 1],  # Rotate left
            "e": [1, -1, 1, -1],  # Rotate right
            # Diagonal movements
            "i": [0, 1, 1, 0],  # Forward-left
            "o": [1, 0, 0, 1],  # Forward-right
            "k": [-1, 0, 0, -1],  # Backward-left
            "l": [0, -1, -1, 0],  # Backward-right
        }

        self.speed_bindings = {
            "z": 1.1,  # Increase speed by 10%
            "c": 0.9,  # Decrease speed by 10%
        }

        self.get_logger().info("Mecanum Teleop Node Started")
        self.print_usage()

    def print_usage(self):
        msg = """
---------------------------
Mecanum Robot Teleop Control
---------------------------
Moving around:
   W         I    O
   A S D     K    L
   
W/S : Forward/Backward
A/D : Strafe Left/Right
Q/E : Rotate Left/Right

I/O/K/L : Diagonal movements

Z/C : Increase/Decrease speed by 10%
X   : Force stop
SPACE : Stop

Current wheel speed: {:.2f} rad/s

CTRL-C to quit
---------------------------
        """.format(self.wheel_speed)
        print(msg)

    def get_key(self):
        """Get keyboard input (non-blocking)"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_wheel_commands(self, wheel_velocities):
        """
        Publish wheel velocity commands
        wheel_velocities: [front_left, front_right, back_left, back_right]
        """
        msg = Float64MultiArray()
        msg.data = [v * self.wheel_speed for v in wheel_velocities]

        self.publisher.publish(msg)

        # Log the command
        self.get_logger().info(
            f"Wheel velocities: FL={msg.data[0]:.2f}, FR={msg.data[1]:.2f}, "
            f"BL={msg.data[2]:.2f}, BR={msg.data[3]:.2f} rad/s"
        )

    def run(self):
        """Main loop"""
        self.settings = termios.tcgetattr(sys.stdin)

        try:
            while rclpy.ok():
                key = self.get_key()

                if key in self.movement_bindings:
                    velocities = self.movement_bindings[key]
                    self.publish_wheel_commands(velocities)

                elif key in self.speed_bindings:
                    self.wheel_speed *= self.speed_bindings[key]
                    self.wheel_speed = max(
                        0.5, min(max_speed, self.wheel_speed)
                    )  # Clamp speed

                    self.get_logger().info(
                        f"Speed updated: {self.wheel_speed:.2f} rad/s"
                    )
                    self.publish_wheel_commands([0, 0, 0, 0])  # Stop after speed change

                elif key in ["x", "X", " "]:
                    # Emergency stop
                    self.publish_wheel_commands([0, 0, 0, 0])
                    self.get_logger().info("STOP")

                elif key == "\x03":  # Ctrl+C
                    break

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        finally:
            # Send stop command before exit
            self.publish_wheel_commands([0, 0, 0, 0])
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)

    try:
        teleop_node = MecanumTeleopKey()
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
