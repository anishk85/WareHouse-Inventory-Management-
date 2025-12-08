#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Range
import gpiod
import time

class PIDController:
    """Simple PID controller for position control"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None
        
        # Anti-windup limits
        self.integral_max = 100.0
        self.integral_min = -100.0
        
    def reset(self):
        """Reset PID controller state"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None
    
    def compute(self, current_value, current_time=None):
        """
        Compute PID output
        Returns: (output, error) tuple
        """
        if current_time is None:
            current_time = time.time()
        
        # Calculate error
        error = self.setpoint - current_value
        
        # Initialize time tracking
        if self.previous_time is None:
            self.previous_time = current_time
            self.previous_error = error
            return 0.0, error
        
        # Calculate dt
        dt = current_time - self.previous_time
        if dt <= 0:
            return 0.0, error
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(self.integral_min, min(self.integral_max, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        d_term = self.kd * derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Update state
        self.previous_error = error
        self.previous_time = current_time
        
        return output, error
    
    def set_gains(self, kp=None, ki=None, kd=None):
        """Update PID gains"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd


class ActuatorControlNode(Node):
    def __init__(self):
        super().__init__('actuator_control_node')
        
        # GPIO pin configuration (BCM numbering)
        self.PWM_PIN = 18  # GPIO 18 for PWM signal
        self.DIR_PIN = 23  # GPIO 23 for direction control
        
        # Declare parameters for PID tuning
        self.declare_parameter('pid_kp', 50.0)  # Proportional gain
        self.declare_parameter('pid_ki', 0.1)   # Integral gain
        self.declare_parameter('pid_kd', 5.0)   # Derivative gain
        self.declare_parameter('target_distance', 1.80)  # 180cm in meters
        self.declare_parameter('distance_tolerance', 0.02)  # ±2cm tolerance
        self.declare_parameter('min_speed', 30)  # Minimum motor speed (0-100)
        self.declare_parameter('max_speed', 100)  # Maximum motor speed
        self.declare_parameter('deadband', 0.01)  # ±1cm deadband (don't move if within this)
        
        # Get parameters
        kp = self.get_parameter('pid_kp').value
        ki = self.get_parameter('pid_ki').value
        kd = self.get_parameter('pid_kd').value
        self.target_distance = self.get_parameter('target_distance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.deadband = self.get_parameter('deadband').value
        
        # Initialize PID controller
        self.pid = PIDController(kp=kp, ki=ki, kd=kd, setpoint=self.target_distance)
        
        # Initialize GPIO using gpiod (version 1.x API)
        try:
            self.chip = gpiod.Chip('gpiochip0')
            
            # Get lines (pins)
            self.pwm_line = self.chip.get_line(self.PWM_PIN)
            self.dir_line = self.chip.get_line(self.DIR_PIN)
            
            # Request lines as outputs with pull-down
            self.pwm_line.request(
                consumer="actuator_pwm", 
                type=gpiod.LINE_REQ_DIR_OUT,
                flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN,
                default_vals=[0]
            )
            self.dir_line.request(
                consumer="actuator_dir", 
                type=gpiod.LINE_REQ_DIR_OUT,
                flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN,
                default_vals=[0]
            )
            
            # Ensure both pins are LOW at startup
            self.pwm_line.set_value(0)
            self.dir_line.set_value(0)
            time.sleep(0.1)
            
            self.get_logger().info('✓ GPIO initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO: {e}')
            raise
        
        # Create subscribers
        self.laser_sub = self.create_subscription(
            Range,
            'laser_distance',
            self.laser_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            String,
            'actuator/command',
            self.command_callback,
            10
        )
        
        # Create publishers
        self.status_pub = self.create_publisher(String, 'actuator/status', 10)
        self.distance_pub = self.create_publisher(Float32, 'actuator/current_distance', 10)
        self.error_pub = self.create_publisher(Float32, 'actuator/position_error', 10)
        self.pid_output_pub = self.create_publisher(Float32, 'actuator/pid_output', 10)
        
        # Create timer for control loop (50 Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # State variables
        self.current_distance = None
        self.last_distance_time = None
        self.control_mode = 'manual'  # 'manual' or 'auto'
        self.is_running = False
        self.current_direction = 'stopped'
        self.at_target = False
        
        # Safety timeout
        self.distance_timeout = 1.0  # seconds
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Actuator Control Node with PID Position Control')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Target Distance: {self.target_distance*100:.1f} cm')
        self.get_logger().info(f'Tolerance: ±{self.distance_tolerance*100:.1f} cm')
        self.get_logger().info(f'Deadband: ±{self.deadband*100:.1f} cm')
        self.get_logger().info(f'PID Gains: Kp={kp}, Ki={ki}, Kd={kd}')
        self.get_logger().info(f'Speed Range: {self.min_speed}-{self.max_speed}%')
        self.get_logger().info('')
        self.get_logger().info('Commands:')
        self.get_logger().info('  "auto"   - Enable PID control to target distance')
        self.get_logger().info('  "manual" - Disable PID control')
        self.get_logger().info('  "up"     - Manual move up')
        self.get_logger().info('  "down"   - Manual move down')
        self.get_logger().info('  "stop"   - Stop movement')
        self.get_logger().info('=' * 60)
        
    def laser_callback(self, msg):
        """Handle laser distance sensor data"""
        # Update current distance (convert to meters if needed)
        self.current_distance = msg.range
        self.last_distance_time = self.get_clock().now()
        
        # Publish current distance
        distance_msg = Float32()
        distance_msg.data = self.current_distance
        self.distance_pub.publish(distance_msg)
        
        # Check if at target in auto mode
        if self.control_mode == 'auto':
            error = abs(self.target_distance - self.current_distance)
            if error <= self.distance_tolerance:
                if not self.at_target:
                    self.at_target = True
                    self.get_logger().info(
                        f'🎯 TARGET REACHED! Distance: {self.current_distance*100:.1f}cm '
                        f'(Target: {self.target_distance*100:.1f}cm)'
                    )
            else:
                self.at_target = False
    
    def control_loop(self):
        """Main control loop running at 50 Hz"""
        if self.control_mode != 'auto':
            return
        
        # Check if we have recent distance data
        if self.current_distance is None or self.last_distance_time is None:
            if self.is_running:
                self.stop_actuator()
                self.get_logger().warn('No laser distance data - stopping for safety')
            return
        
        # Check distance timeout
        time_since_update = (self.get_clock().now() - self.last_distance_time).nanoseconds / 1e9
        if time_since_update > self.distance_timeout:
            if self.is_running:
                self.stop_actuator()
                self.get_logger().warn(f'Distance data timeout ({time_since_update:.1f}s) - stopping')
            return
        
        # Check if distance is valid
        if self.current_distance < 0.01 or self.current_distance > 4.0:
            if self.is_running:
                self.stop_actuator()
                self.get_logger().warn(f'Invalid distance reading: {self.current_distance:.3f}m')
            return
        
        # Compute PID output
        pid_output, error = self.pid.compute(self.current_distance)
        
        # Publish PID data
        error_msg = Float32()
        error_msg.data = error
        self.error_pub.publish(error_msg)
        
        pid_msg = Float32()
        pid_msg.data = pid_output
        self.pid_output_pub.publish(pid_msg)
        
        # Check deadband
        if abs(error) < self.deadband:
            if self.is_running:
                self.stop_actuator()
                self.get_logger().info(
                    f'✓ Within deadband (±{self.deadband*100:.1f}cm), stopping'
                )
            return
        
        # Determine direction and speed
        if error > 0:
            # Current distance < target, need to move UP (increase distance)
            direction = 'up'
        else:
            # Current distance > target, need to move DOWN (decrease distance)
            direction = 'down'
        
        # Map PID output to speed (with min/max limits)
        speed = abs(pid_output)
        speed = max(self.min_speed, min(self.max_speed, speed))
        
        # Apply movement
        if direction == 'up':
            self.move_up_with_speed(speed)
        else:
            self.move_down_with_speed(speed)
        
        # Log status periodically
        if hasattr(self, '_last_log_time'):
            if time.time() - self._last_log_time > 1.0:  # Log every 1 second
                self.get_logger().info(
                    f'📍 Distance: {self.current_distance*100:6.1f}cm | '
                    f'Error: {error*100:+6.1f}cm | '
                    f'PID: {pid_output:+6.1f} | '
                    f'Speed: {speed:3.0f}% | '
                    f'Dir: {direction}'
                )
                self._last_log_time = time.time()
        else:
            self._last_log_time = time.time()
    
    def command_callback(self, msg):
        """Handle command messages"""
        command = msg.data.lower()
        
        if command == 'auto':
            self.enable_auto_mode()
        elif command == 'manual':
            self.enable_manual_mode()
        elif command == 'up':
            if self.control_mode == 'manual':
                self.lift_up()
            else:
                self.get_logger().warn('In AUTO mode - switch to manual first')
        elif command == 'down':
            if self.control_mode == 'manual':
                self.lift_down()
            else:
                self.get_logger().warn('In AUTO mode - switch to manual first')
        elif command == 'stop':
            self.stop_actuator()
        else:
            self.get_logger().warn(f'Unknown command: {command}')
    
    def enable_auto_mode(self):
        """Enable automatic PID control"""
        self.control_mode = 'auto'
        self.pid.reset()
        self.at_target = False
        self.get_logger().info(
            f'🤖 AUTO mode enabled - Moving to target: {self.target_distance*100:.1f}cm'
        )
        self.publish_status('auto_mode_enabled')
    
    def enable_manual_mode(self):
        """Enable manual control"""
        self.stop_actuator()
        self.control_mode = 'manual'
        self.get_logger().info('👤 MANUAL mode enabled')
        self.publish_status('manual_mode_enabled')
    
    def move_up_with_speed(self, speed):
        """Move up with specified speed (0-100)"""
        # For now, we use simple on/off control
        # TODO: Implement actual PWM for variable speed
        if not self.is_running or self.current_direction != 'up':
            self._set_direction_and_pwm('up')
    
    def move_down_with_speed(self, speed):
        """Move down with specified speed (0-100)"""
        # For now, we use simple on/off control
        # TODO: Implement actual PWM for variable speed
        if not self.is_running or self.current_direction != 'down':
            self._set_direction_and_pwm('down')
    
    def _set_direction_and_pwm(self, direction):
        """Internal method to set direction and enable PWM"""
        # Stop PWM first
        self.pwm_line.set_value(0)
        time.sleep(0.01)
        
        # Set direction (INVERTED: LOW=UP, HIGH=DOWN)
        if direction == 'up':
            self.dir_line.set_value(0)
        else:  # down
            self.dir_line.set_value(1)
        
        time.sleep(0.02)
        
        # Enable PWM
        self.pwm_line.set_value(1)
        
        self.is_running = True
        self.current_direction = direction
    
    def lift_up(self):
        """Manual move up"""
        self._set_direction_and_pwm('up')
        self.get_logger().info('🔼 Manual UP')
        self.publish_status('manual_up')
    
    def lift_down(self):
        """Manual move down"""
        self._set_direction_and_pwm('down')
        self.get_logger().info('🔽 Manual DOWN')
        self.publish_status('manual_down')
    
    def stop_actuator(self):
        """Stop actuator movement"""
        # Stop PWM
        self.pwm_line.set_value(0)
        time.sleep(0.02)
        
        # Reset direction
        self.dir_line.set_value(0)
        
        self.is_running = False
        old_dir = self.current_direction
        self.current_direction = 'stopped'
        
        if old_dir != 'stopped':
            self.get_logger().info(f'⏹️  STOPPED (was {old_dir})')
        
        self.publish_status('stopped')
    
    def publish_status(self, status):
        """Publish current status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def cleanup(self):
        """Cleanup GPIO on shutdown"""
        self.get_logger().info('Cleaning up GPIO...')
        try:
            self.is_running = False
            
            # Proper shutdown sequence
            self.pwm_line.set_value(0)
            time.sleep(0.02)
            self.dir_line.set_value(0)
            time.sleep(0.02)
            
            self.pwm_line.release()
            self.dir_line.release()
            self.chip.close()
            self.get_logger().info('GPIO cleanup complete')
        except Exception as e:
            self.get_logger().error(f'Cleanup error: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = ActuatorControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt detected')
    except Exception as e:
        if node:
            node.get_logger().error(f'Error: {e}')
        else:
            print(f'Failed to initialize node: {e}')
    finally:
        if node:
            node.cleanup()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()