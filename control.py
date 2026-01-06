#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import RPi.GPIO as GPIO
import time

class ActuatorControlNode(Node):
    def __init__(self):
        super().__init__('actuator_control_node')
        
        # GPIO pin configuration
        self.PWM_PIN = 18  # GPIO 18 for PWM signal
        self.DIR_PIN = 23  # GPIO 23 for direction control
        
        # PWM configuration
        self.PWM_FREQUENCY = 1000  # 1kHz PWM frequency
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.PWM_PIN, GPIO.OUT)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        
        # Setup PWM
        self.pwm = GPIO.PWM(self.PWM_PIN, self.PWM_FREQUENCY)
        self.pwm.start(0)  # Start with 0% duty cycle
        
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
        
        # Current state
        self.current_speed = 0
        self.current_direction = 'stopped'
        
        self.get_logger().info('Actuator Control Node initialized')
        self.get_logger().info(f'PWM Pin: {self.PWM_PIN}, DIR Pin: {self.DIR_PIN}')
        
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
        """Handle speed messages (0-100)"""
        speed = max(0, min(100, msg.data))  # Clamp between 0-100
        self.current_speed = speed
        self.pwm.ChangeDutyCycle(speed)
        self.get_logger().info(f'Speed set to: {speed}%')
    
    def lift_up(self):
        """Move actuator up"""
        GPIO.output(self.DIR_PIN, GPIO.HIGH)  # Set direction to UP
        if self.current_speed == 0:
            self.current_speed = 50  # Default speed if not set
        self.pwm.ChangeDutyCycle(self.current_speed)
        self.current_direction = 'up'
        
        self.get_logger().info(f'Moving UP at {self.current_speed}% speed')
        self.publish_status('moving_up')
    
    def lift_down(self):
        """Move actuator down"""
        GPIO.output(self.DIR_PIN, GPIO.LOW)  # Set direction to DOWN
        if self.current_speed == 0:
            self.current_speed = 50  # Default speed if not set
        self.pwm.ChangeDutyCycle(self.current_speed)
        self.current_direction = 'down'
        
        self.get_logger().info(f'Moving DOWN at {self.current_speed}% speed')
        self.publish_status('moving_down')
    
    def stop_actuator(self):
        """Stop actuator movement"""
        self.pwm.ChangeDutyCycle(0)
        self.current_direction = 'stopped'
        
        self.get_logger().info('Actuator STOPPED')
        self.publish_status('stopped')
    
    def publish_status(self, status):
        """Publish current status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def cleanup(self):
        """Cleanup GPIO on shutdown"""
        self.get_logger().info('Cleaning up GPIO...')
        self.pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    
    node = ActuatorControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
