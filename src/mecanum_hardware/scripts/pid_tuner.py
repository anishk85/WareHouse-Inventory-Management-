#!/usr/bin/env python3
"""
PID Tuning Helper for Actuator Control
Provides interactive CLI for tuning PID parameters in real-time
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import sys
import select
import tty
import termios

class PIDTuner(Node):
    def __init__(self):
        super().__init__('pid_tuner')
        
        # Subscribe to status topics
        self.distance_sub = self.create_subscription(
            Float32,
            'actuator/current_distance',
            self.distance_callback,
            10
        )
        
        self.error_sub = self.create_subscription(
            Float32,
            'actuator/position_error',
            self.error_callback,
            10
        )
        
        self.pid_output_sub = self.create_subscription(
            Float32,
            'actuator/pid_output',
            self.pid_output_callback,
            10
        )
        
        self.status_sub = self.create_subscription(
            String,
            'actuator/status',
            self.status_callback,
            10
        )
        
        # Publisher for commands
        self.command_pub = self.create_publisher(String, 'actuator/command', 10)
        
        # Current values
        self.current_distance = None
        self.current_error = None
        self.current_pid_output = None
        self.current_status = 'unknown'
        
        # PID parameters (get from node)
        self.kp = 50.0
        self.ki = 0.1
        self.kd = 5.0
        self.target = 1.80
        
        # Display timer
        self.display_timer = self.create_timer(0.5, self.display_status)
        
        self.get_logger().info('PID Tuner started')
        self.print_help()
    
    def distance_callback(self, msg):
        self.current_distance = msg.data
    
    def error_callback(self, msg):
        self.current_error = msg.data
    
    def pid_output_callback(self, msg):
        self.current_pid_output = msg.data
    
    def status_callback(self, msg):
        self.current_status = msg.data
    
    def display_status(self):
        """Display current system status"""
        print('\033[2J\033[H')  # Clear screen
        print('=' * 70)
        print('  PID TUNING DASHBOARD')
        print('=' * 70)
        print()
        
        # Current state
        print('CURRENT STATE:')
        print('-' * 70)
        if self.current_distance is not None:
            print(f'  Distance:    {self.current_distance*100:7.2f} cm  (Target: {self.target*100:.1f} cm)')
        else:
            print('  Distance:    --- cm (No data)')
        
        if self.current_error is not None:
            print(f'  Error:       {self.current_error*100:+7.2f} cm')
        else:
            print('  Error:       --- cm')
        
        if self.current_pid_output is not None:
            print(f'  PID Output:  {self.current_pid_output:+7.2f}')
        else:
            print('  PID Output:  ---')
        
        print(f'  Status:      {self.current_status}')
        print()
        
        # PID parameters
        print('PID PARAMETERS:')
        print('-' * 70)
        print(f'  Kp (Proportional): {self.kp:8.3f}  [Q/W to decrease/increase]')
        print(f'  Ki (Integral):     {self.ki:8.3f}  [A/S to decrease/increase]')
        print(f'  Kd (Derivative):   {self.kd:8.3f}  [Z/X to decrease/increase]')
        print(f'  Target Distance:   {self.target*100:6.1f} cm  [1/2 to decrease/increase]')
        print()
        
        # Error visualization
        if self.current_error is not None:
            self.print_error_bar(self.current_error)
        
        print()
        print('CONTROLS:')
        print('-' * 70)
        print('  [Space] Start AUTO mode    [M] Manual mode    [P] Stop')
        print('  [H]     Show this help     [R] Reset PID      [Esc] Quit')
        print('=' * 70)
    
    def print_error_bar(self, error_m):
        """Print visual error bar"""
        error_cm = error_m * 100
        max_display = 50  # cm
        
        # Calculate bar position
        bar_width = 40
        center = bar_width // 2
        
        if abs(error_cm) > max_display:
            pos = center + (bar_width // 2 if error_cm > 0 else -(bar_width // 2))
        else:
            pos = center + int((error_cm / max_display) * (bar_width // 2))
        
        pos = max(0, min(bar_width - 1, pos))
        
        # Build bar
        bar = ['-'] * bar_width
        bar[center] = '|'  # Target
        bar[pos] = '●'      # Current
        
        print('ERROR VISUALIZATION:')
        print('  Too Close' + ' ' * 15 + 'Target' + ' ' * 15 + 'Too Far')
        print('  [' + ''.join(bar) + ']')
        print(f'  {error_cm:+.1f} cm from target')
    
    def print_help(self):
        """Print help message"""
        print('\n' + '=' * 70)
        print('  PID TUNING GUIDE')
        print('=' * 70)
        print()
        print('TUNING PROCESS:')
        print('  1. Start with Kp only (set Ki=0, Kd=0)')
        print('  2. Increase Kp until system oscillates')
        print('  3. Reduce Kp to 60% of oscillation value')
        print('  4. Add Kd to reduce overshoot')
        print('  5. Add small Ki if steady-state error exists')
        print()
        print('PARAMETER EFFECTS:')
        print('  Kp: Response speed (too high = oscillation)')
        print('  Ki: Eliminates steady-state error (too high = instability)')
        print('  Kd: Damping (reduces overshoot and oscillation)')
        print()
        print('Press any key to continue...')
        print('=' * 70)
    
    def send_command(self, command):
        """Send command to actuator"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'Command sent: {command}')
    
    def adjust_parameter(self, param, delta):
        """Adjust PID parameter"""
        if param == 'kp':
            self.kp = max(0, self.kp + delta)
            self.get_logger().info(f'Kp adjusted to {self.kp}')
        elif param == 'ki':
            self.ki = max(0, self.ki + delta)
            self.get_logger().info(f'Ki adjusted to {self.ki}')
        elif param == 'kd':
            self.kd = max(0, self.kd + delta)
            self.get_logger().info(f'Kd adjusted to {self.kd}')
        elif param == 'target':
            self.target = max(0.1, min(4.0, self.target + delta))
            self.get_logger().info(f'Target adjusted to {self.target*100:.1f}cm')
        
        # TODO: Actually send parameter update to node
        # This would require dynamic reconfigure or parameter services


def main(args=None):
    rclpy.init(args=args)
    
    tuner = PIDTuner()
    
    # Set terminal to non-blocking mode
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        tty.setcbreak(sys.stdin.fileno())
        
        print('Starting PID Tuner...')
        print('Press H for help')
        
        while rclpy.ok():
            # Spin once to process callbacks
            rclpy.spin_once(tuner, timeout_sec=0.1)
            
            # Check for keyboard input (non-blocking)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                
                # Handle key presses
                if key == ' ':  # Space - start auto mode
                    tuner.send_command('auto')
                elif key.lower() == 'm':
                    tuner.send_command('manual')
                elif key.lower() == 'p':
                    tuner.send_command('stop')
                elif key.lower() == 'q':  # Decrease Kp
                    tuner.adjust_parameter('kp', -5.0)
                elif key.lower() == 'w':  # Increase Kp
                    tuner.adjust_parameter('kp', 5.0)
                elif key.lower() == 'a':  # Decrease Ki
                    tuner.adjust_parameter('ki', -0.01)
                elif key.lower() == 's':  # Increase Ki
                    tuner.adjust_parameter('ki', 0.01)
                elif key.lower() == 'z':  # Decrease Kd
                    tuner.adjust_parameter('kd', -0.5)
                elif key.lower() == 'x':  # Increase Kd
                    tuner.adjust_parameter('kd', 0.5)
                elif key == '1':  # Decrease target
                    tuner.adjust_parameter('target', -0.05)
                elif key == '2':  # Increase target
                    tuner.adjust_parameter('target', 0.05)
                elif key.lower() == 'h':
                    tuner.print_help()
                elif key == '\x1b':  # Escape
                    break
                
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        tuner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()