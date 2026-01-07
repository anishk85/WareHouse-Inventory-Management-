#!/usr/bin/env python3
"""
Simple ESP32 Lift Control Node
Sends height commands to ESP32 via TCP when triggered by bool messages
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
import socket
import threading


class SimpleLiftControl(Node):
    def __init__(self):
        super().__init__('simple_lift_control')
        
        # Parameters
        self.declare_parameter('esp32_ip', '192.168.1.100')
        self.declare_parameter('esp32_port', 8080)
        self.declare_parameter('lift_up_height', 500.0)      # Height when lifting up
        self.declare_parameter('lift_down_height', 100.0)    # Height when lowering down
        
        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.esp32_port = self.get_parameter('esp32_port').value
        self.up_height = self.get_parameter('lift_up_height').value
        self.down_height = self.get_parameter('lift_down_height').value
        
        # TCP socket
        self.socket = None
        self.socket_lock = threading.Lock()
        
        # Subscribers
        self.sub_lift_up = self.create_subscription(
            Bool, 'lift_up', self.callback_lift_up, 10)
        self.sub_lift_down = self.create_subscription(
            Bool, 'lift_down', self.callback_lift_down, 10)
        self.sub_set_height = self.create_subscription(
            Float32, 'lift_set_height', self.callback_set_height, 10)
        self.sub_stop = self.create_subscription(
            Bool, 'lift_stop', self.callback_stop, 10)
        
        # Publisher for status
        self.pub_status = self.create_publisher(String, 'lift_status', 10)
        
        # Connect to ESP32
        self.connect_esp32()
        
        self.get_logger().info(f'Simple Lift Control Started')
        self.get_logger().info(f'ESP32: {self.esp32_ip}:{self.esp32_port}')
        self.get_logger().info(f'Up height: {self.up_height} mm, Down height: {self.down_height} mm')
    
    def connect_esp32(self):
        """Connect to ESP32 TCP server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(3.0)
            self.socket.connect((self.esp32_ip, self.esp32_port))
            
            # Read welcome message
            welcome = self.socket.recv(1024).decode('utf-8').strip()
            self.get_logger().info(f'Connected to ESP32: {welcome}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            self.socket = None
    
    def send_command(self, command):
        """Send command to ESP32"""
        if not self.socket:
            self.get_logger().warn('Not connected to ESP32')
            return False
        
        try:
            with self.socket_lock:
                self.socket.sendall((command + '\n').encode('utf-8'))
                self.socket.settimeout(1.0)
                response = self.socket.recv(1024).decode('utf-8').strip()
                
                self.get_logger().info(f'Command: {command} | Response: {response}')
                
                # Publish status
                msg = String()
                msg.data = response
                self.pub_status.publish(msg)
                
                return True
                
        except Exception as e:
            self.get_logger().error(f'Command failed: {e}')
            return False
    
    def callback_lift_up(self, msg):
        """Lift up when True is received"""
        if msg.data:
            self.get_logger().info(f'LIFT UP to {self.up_height} mm')
            self.send_command(f'HEIGHT:{self.up_height}')
    
    def callback_lift_down(self, msg):
        """Lift down when True is received"""
        if msg.data:
            self.get_logger().info(f'LIFT DOWN to {self.down_height} mm')
            self.send_command(f'HEIGHT:{self.down_height}')
    
    def callback_set_height(self, msg):
        """Set specific height"""
        height = msg.data
        self.get_logger().info(f'SET HEIGHT to {height} mm')
        self.send_command(f'HEIGHT:{height}')
    
    def callback_stop(self, msg):
        """Emergency stop"""
        if msg.data:
            self.get_logger().warn('EMERGENCY STOP!')
            self.send_command('STOP')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLiftControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.socket:
            node.socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()