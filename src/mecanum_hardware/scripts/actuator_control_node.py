#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import serial
import time

class ActuatorControlNode(Node):
    def __init__(self):
        super().__init__('actuator_control_node')
        
        # ESP32 Serial Configuration
        self.ESP32_PORT = '/dev/ttyUSB3'
        self.ESP32_BAUD = 115200
        self.ser = None
        self.connected = False
        
        # Connect to ESP32
        self.connect_esp32()
        
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
        
        # Subscribe to joystick buttons
        self.joy_sub = self.create_subscription(
            String,
            'joy_buttons',
            self.joy_button_callback,
            10
        )
        
        # Create publisher for status
        self.status_pub = self.create_publisher(String, 'actuator/status', 10)
        
        # Current state - ALWAYS RUN AT FULL SPEED
        self.current_speed = 255  # FULL SPEED (0-255)
        self.current_direction = 'stopped'
        self.is_running = False
        
        # Button state for continuous control
        self.continuous_up = False  # X button pressed once = continuous UP
        self.continuous_down = False  # Y button pressed once = continuous DOWN
        self.plus_button_held = False  # Plus button hold state
        
        # Track last plus button direction for hold behavior
        self.last_plus_direction = None
        
        # Watchdog: Track last command time to detect stale button state
        self.last_command_time = time.time()
        
        # Create timer for continuous command sending (50Hz = 20ms)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Actuator Control Node initialized (ESP32 Serial)')
        self.get_logger().info(f'ESP32 Port: {self.ESP32_PORT} @ {self.ESP32_BAUD} baud')
        self.get_logger().info('Operating at FULL SPEED (255) always')
        self.get_logger().info('')
        self.get_logger().info('=== CONTROL SCHEME ===')
        self.get_logger().info('X button:    Press ONCE → Continuous UP (until RT/LT pressed)')
        self.get_logger().info('Y button:    Press ONCE → Continuous DOWN (until RT/LT pressed)')
        self.get_logger().info('RT/LT:       STOP movement')
        self.get_logger().info('Plus button: HOLD to move in last direction (stops when released)')
        self.get_logger().info('Commands at 50Hz while active')
        self.get_logger().info('')
        
    def connect_esp32(self):
        """Establish serial connection to ESP32"""
        try:
            self.ser = serial.Serial(
                port=self.ESP32_PORT,
                baudrate=self.ESP32_BAUD,
                timeout=0.5,
                write_timeout=0.5
            )
            time.sleep(1)
            self.connected = True
            self.get_logger().info(f'✓ Connected to ESP32 on {self.ESP32_PORT}')
            
            # Clear any initial garbage data
            try:
                while self.ser.in_waiting > 0:
                    msg = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if msg:
                        self.get_logger().info(f'ESP32 init: {msg}')
            except Exception as e:
                self.get_logger().debug(f'No initial ESP32 messages: {e}')
                    
        except serial.SerialException as e:
            self.get_logger().error(f'✗ Failed to connect to ESP32: {e}')
            self.get_logger().error('Check:')
            self.get_logger().error('  1. ESP32 connected to /dev/ttyUSB3')
            self.get_logger().error('  2. Permissions: sudo usermod -a -G dialout $USER')
            self.get_logger().error('  3. Port correct: ls /dev/ttyUSB*')
            self.connected = False
    
    def send_esp32_command(self, pwm, direction):
        """
        Send motor command to ESP32
        Format: "PWM,DIR\n"
        PWM: 0-255
        DIR: 0=reverse, 1=forward
        """
        if not self.connected:
            return False
        
        if not self.ser or not self.ser.is_open:
            self.get_logger().error('Serial port is closed')
            return False
        
        try:
            pwm = max(0, min(255, int(pwm)))
            direction = 1 if direction else 0
            
            cmd = f"{pwm},{direction}\n"
            
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
            self.get_logger().debug(f'Sent to ESP32: {cmd.strip()}')
            return True
            
        except serial.SerialException as e:
            self.get_logger().error(f'Error sending to ESP32: {e}')
            self.connected = False
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            return False
    
    def command_callback(self, msg):
        """
        Handle command messages from Plus button: 'up', 'down', 'stop'
        Plus button behavior: commands come as 'up' or 'down' repeatedly while held
        """
        command = msg.data.lower()
        
        if command == 'up':
            # Plus button sending UP - enable hold mode
            self.plus_button_held = True
            self.last_plus_direction = 'up'
            self.continuous_up = False  # Cancel continuous modes
            self.continuous_down = False
            self.get_logger().debug('Plus: UP (hold active)')
            
        elif command == 'down':
            # Plus button sending DOWN - enable hold mode
            self.plus_button_held = True
            self.last_plus_direction = 'down'
            self.continuous_up = False  # Cancel continuous modes
            self.continuous_down = False
            self.get_logger().debug('Plus: DOWN (hold active)')
            
        elif command == 'stop':
            # Plus button released or explicit stop
            self.plus_button_held = False
            self.last_plus_direction = None
            self.get_logger().info('⏹️ Plus button STOP')
        else:
            self.get_logger().warn(f'Unknown command: {command}')
    
    def speed_callback(self, msg):
        """Handle speed messages (0-100) - currently ignored, always full speed"""
        speed = max(0, min(100, msg.data))
        self.get_logger().info(f'Speed command received ({speed}%), but operating at FULL SPEED always')
    
    def joy_button_callback(self, msg):
        """
        Handle joystick button presses:
        - X button (press once): Start continuous UP until RT/LT pressed
        - Y button (press once): Start continuous DOWN until RT/LT pressed  
        - RT button: STOP movement
        - LT button: STOP movement
        
        Expected format: "BUTTON:STATE"
        Examples: "X:1", "X:0", "Y:1", "RT:1", "LT:1"
        """
        button_data = msg.data.strip()
        
        if ':' not in button_data:
            self.get_logger().warn(f'Invalid button format: "{button_data}"')
            return
        
        try:
            button, state = button_data.split(':')
            button = button.strip().upper()
            state = int(state.strip())
        except ValueError:
            self.get_logger().warn(f'Invalid button data: "{button_data}"')
            return
        
        # X button - Press once for continuous UP
        if button == 'X':
            if state == 1:  # Button PRESSED
                self.continuous_up = True
                self.continuous_down = False
                self.plus_button_held = False  # Cancel plus button mode
                self.last_command_time = time.time()
                self.get_logger().info('🔼 X pressed → CONTINUOUS UP (press RT/LT to stop)')
            # Note: Releasing X does NOT stop - only RT/LT stops
                
        # Y button - Press once for continuous DOWN
        elif button == 'Y':
            if state == 1:  # Button PRESSED
                self.continuous_down = True
                self.continuous_up = False
                self.plus_button_held = False  # Cancel plus button mode
                self.last_command_time = time.time()
                self.get_logger().info('🔽 Y pressed → CONTINUOUS DOWN (press RT/LT to stop)')
            # Note: Releasing Y does NOT stop - only RT/LT stops
                
        # RT button - STOP
        elif button == 'RT':
            if state == 1:  # Button PRESSED
                self.continuous_up = False
                self.continuous_down = False
                self.plus_button_held = False
                self.get_logger().info('⏹️ RT pressed → STOP')
                
        # LT button - STOP
        elif button == 'LT':
            if state == 1:  # Button PRESSED
                self.continuous_up = False
                self.continuous_down = False
                self.plus_button_held = False
                self.get_logger().info('⏹️ LT pressed → STOP')
        else:
            self.get_logger().debug(f'Unhandled button: "{button}"')
    
    def control_loop(self):
        """
        Timer callback that runs at 50Hz (every 20ms)
        Priority order:
        1. Plus button hold mode (while button held)
        2. Continuous UP mode (X button pressed once)
        3. Continuous DOWN mode (Y button pressed once)
        4. Stop (none active)
        """
        # Priority 1: Plus button hold mode
        if self.plus_button_held and self.last_plus_direction:
            if self.last_plus_direction == 'up':
                self.send_esp32_command(self.current_speed, 1)
                if self.current_direction != 'plus_up':
                    self.current_direction = 'plus_up'
                    self.is_running = True
                    self.publish_status('plus_up')
                    self.get_logger().info('🔼 Plus button HELD → Moving UP')
            else:  # down
                self.send_esp32_command(self.current_speed, 0)
                if self.current_direction != 'plus_down':
                    self.current_direction = 'plus_down'
                    self.is_running = True
                    self.publish_status('plus_down')
                    self.get_logger().info('🔽 Plus button HELD → Moving DOWN')
        
        # Priority 2: Continuous UP mode (X button)
        elif self.continuous_up:
            self.send_esp32_command(self.current_speed, 1)
            if self.current_direction != 'continuous_up':
                self.current_direction = 'continuous_up'
                self.is_running = True
                self.publish_status('continuous_up')
                self.get_logger().info('🔼 X button → Continuous UP active')
                
        # Priority 3: Continuous DOWN mode (Y button)
        elif self.continuous_down:
            self.send_esp32_command(self.current_speed, 0)
            if self.current_direction != 'continuous_down':
                self.current_direction = 'continuous_down'
                self.is_running = True
                self.publish_status('continuous_down')
                self.get_logger().info('🔽 Y button → Continuous DOWN active')
                
        # Priority 4: STOP (no active command)
        else:
            if self.is_running:
                self.send_esp32_command(0, 0)
                old_direction = self.current_direction
                self.current_direction = 'stopped'
                self.is_running = False
                self.publish_status('stopped')
                self.get_logger().info(f'⏹️ STOPPED (was: {old_direction})')
    
    def publish_status(self, status):
        """Publish current status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def cleanup(self):
        """Cleanup serial connection on shutdown"""
        self.get_logger().info('Cleaning up actuator control...')
        try:
            # Stop motor safely
            if self.connected and self.ser and self.ser.is_open:
                try:
                    self.send_esp32_command(0, 0)  # Stop
                    time.sleep(0.05)
                except Exception as e:
                    self.get_logger().error(f'Error stopping motor: {e}')
                
                try:
                    self.ser.close()
                except Exception as e:
                    self.get_logger().error(f'Error closing serial port: {e}')
            
            self.connected = False
            self.get_logger().info('✓ Actuator cleanup complete')
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
            node.get_logger().error(f'Error in node: {e}')
            import traceback
            node.get_logger().error(traceback.format_exc())
        else:
            print(f'Failed to initialize node: {e}')
            import traceback
            traceback.print_exc()
    finally:
        if node:
            try:
                node.cleanup()
            except Exception as e:
                print(f'Cleanup error: {e}')
            try:
                node.destroy_node()
            except Exception as e:
                print(f'Destroy node error: {e}')
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()