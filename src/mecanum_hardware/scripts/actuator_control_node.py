#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import gpiod
import time

class ActuatorControlNode(Node):
    def __init__(self):
        super().__init__('actuator_control_node')
        
        # GPIO pin configuration (BCM numbering)
        self.PWM_PIN = 18  # GPIO 18 for PWM signal
        self.DIR_PIN = 23  # GPIO 23 for direction control
        
        # Initialize GPIO using gpiod (version 1.x API)
        try:
            self.chip = gpiod.Chip('gpiochip0')
            
            # Get lines (pins)
            self.pwm_line = self.chip.get_line(self.PWM_PIN)
            self.dir_line = self.chip.get_line(self.DIR_PIN)
            
            # Request lines as outputs with explicit initial states
            # IMPORTANT: Start with PWM LOW and DIR LOW to prevent glitches
            self.pwm_line.request(
                consumer="actuator_pwm", 
                type=gpiod.LINE_REQ_DIR_OUT,
                flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN,  # Pull-down to prevent floating
                default_vals=[0]
            )
            self.dir_line.request(
                consumer="actuator_dir", 
                type=gpiod.LINE_REQ_DIR_OUT,
                flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN,  # Pull-down to prevent floating
                default_vals=[0]
            )
            
            # Ensure both pins are LOW at startup
            self.pwm_line.set_value(0)
            self.dir_line.set_value(0)
            time.sleep(0.1)  # Give hardware time to stabilize
            
            self.get_logger().info('GPIO initialized successfully using gpiod with pull-down resistors')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO: {e}')
            self.get_logger().error('Make sure /dev/gpiochip0 is accessible')
            raise
        
        # Create subscribers
        self.command_sub = self.create_subscription(
            String,
            'actuator/command',
            self.command_callback,
            10
        )
        
        self.speed_sub = self.create_subscription(
            Int32,
            'actuator/speed',
            self.speed_callback,
            10
        )
        
        # Create publisher for status
        self.status_pub = self.create_publisher(String, 'actuator/status', 10)
        
        # Current state - ALWAYS RUN AT FULL SPEED
        self.current_speed = 100  # FULL SPEED ALWAYS
        self.current_direction = 'stopped'
        self.is_running = False
        
        self.get_logger().info('Actuator Control Node initialized')
        self.get_logger().info(f'PWM Pin: GPIO{self.PWM_PIN}, DIR Pin: GPIO{self.DIR_PIN}')
        self.get_logger().info('Operating at FULL SPEED (100%) always')
        self.get_logger().info('Commands: "up", "down", "stop"')
        self.get_logger().info('Motor will run continuously until "stop" command is sent')
        self.get_logger().info('')
        self.get_logger().info('HARDWARE CHECKS:')
        self.get_logger().info('  ✓ Ensure RPi GND connected to motor driver GND')
        self.get_logger().info('  ✓ Motor driver has separate power supply (NOT from RPi)')
        self.get_logger().info('  ✓ Use short, thick wires for power connections')
        
    def command_callback(self, msg):
        """Handle command messages: 'up', 'down', 'stop'"""
        command = msg.data.lower()
        
        if command == 'up':
            self.lift_up()
        elif command == 'down':
            self.lift_down()
        elif command == 'stop':
            self.stop_actuator()
        else:
            self.get_logger().warn(f'Unknown command: {command}')
    
    def speed_callback(self, msg):
        """Handle speed messages (0-100) - currently ignored, always full speed"""
        speed = max(0, min(100, msg.data))
        self.get_logger().info(f'Speed command received ({speed}%), but operating at FULL SPEED always')
    
    def lift_up(self):
        """Move actuator up at FULL SPEED - runs continuously until stop"""
        # CRITICAL SEQUENCE TO PREVENT DIRECTION FLIP:
        # 1. Ensure PWM is completely OFF
        self.pwm_line.set_value(0)
        time.sleep(0.01)  # 10ms delay - longer to ensure motor stops
        
        # 2. Set direction pin to UP - INVERTED: LOW = UP
        self.dir_line.set_value(0)  
        time.sleep(0.02)  # 20ms delay - allow direction to stabilize
        
        # 3. Enable PWM at FULL SPEED
        self.pwm_line.set_value(1)
        
        self.is_running = True
        self.current_direction = 'up'
        
        self.get_logger().info('🔼 Moving UP at FULL SPEED (100%) - RUNNING CONTINUOUSLY')
        self.get_logger().info('   Send "stop" command to stop the motor')
        self.publish_status('moving_up')
    
    def lift_down(self):
        """Move actuator down at FULL SPEED - runs continuously until stop"""
        # CRITICAL SEQUENCE TO PREVENT DIRECTION FLIP:
        # 1. Ensure PWM is completely OFF
        self.pwm_line.set_value(0)
        time.sleep(0.01)  # 10ms delay - longer to ensure motor stops
        
        # 2. Set direction pin to DOWN - INVERTED: HIGH = DOWN
        self.dir_line.set_value(1) 
        time.sleep(0.02)  # 20ms delay - allow direction to stabilize
        
        # 3. Enable PWM at FULL SPEED
        self.pwm_line.set_value(1)
        
        self.is_running = True
        self.current_direction = 'down'
        
        self.get_logger().info('🔽 Moving DOWN at FULL SPEED (100%) - RUNNING CONTINUOUSLY')
        self.get_logger().info('   Send "stop" command to stop the motor')
        self.publish_status('moving_down')
    
    def stop_actuator(self):
        """Stop actuator movement"""
        old_direction = self.current_direction
        
        # PROPER STOP SEQUENCE:
        # 1. Stop PWM immediately
        self.pwm_line.set_value(0)
        time.sleep(0.02)  # 20ms delay - allow motor to coast down
        
        # 2. Reset direction to default (LOW)
        self.dir_line.set_value(0)
        
        self.is_running = False
        self.current_direction = 'stopped'
        
        self.get_logger().info(f'⏹️  Actuator STOPPED (was moving {old_direction})')
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