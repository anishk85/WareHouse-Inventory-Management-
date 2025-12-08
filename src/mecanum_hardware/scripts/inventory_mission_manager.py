#!/usr/bin/env python3
"""
Inventory Mission Manager
Orchestrates autonomous inventory scanning missions:
1. Navigate to waypoints
2. Control actuator (lift up)
3. Trigger QR detection
4. Wait for scans
5. Lower actuator
6. Move to next waypoint
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32
from warehouse_rover_msgs.srv import GetWaypoints
from tf_transformations import quaternion_from_euler
import time
import math


class InventoryMissionManager(Node):
    """
    Manages autonomous inventory scanning missions
    
    States:
        IDLE -> NAVIGATING -> POSITIONING -> LIFTING -> SCANNING -> LOWERING -> NEXT
    """
    
    # Mission states
    STATE_IDLE = 'IDLE'
    STATE_NAVIGATING = 'NAVIGATING'
    STATE_POSITIONING = 'POSITIONING'
    STATE_LIFTING = 'LIFTING'
    STATE_SCANNING = 'SCANNING'
    STATE_LOWERING = 'LOWERING'
    STATE_COMPLETED = 'COMPLETED'
    STATE_FAILED = 'FAILED'
    
    def __init__(self):
        super().__init__('inventory_mission_manager')
        
        # Parameters
        self.declare_parameter('scan_duration', 50.0)  # 50 seconds per rack
        self.declare_parameter('lift_duration', 5.0)   # 5 seconds to lift
        self.declare_parameter('lower_duration', 5.0)  # 5 seconds to lower
        self.declare_parameter('actuator_speed', 50)   # PWM speed for actuator
        self.declare_parameter('waypoints_file', '/tmp/warehouse_waypoints.json')
        
        self.scan_duration = self.get_parameter('scan_duration').value
        self.lift_duration = self.get_parameter('lift_duration').value
        self.lower_duration = self.get_parameter('lower_duration').value
        self.actuator_speed = self.get_parameter('actuator_speed').value
        
        # State
        self.current_state = self.STATE_IDLE
        self.waypoints = []
        self.current_waypoint_index = 0
        self.mission_start_time = None
        self.state_start_time = None
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Service client for waypoints
        self.waypoint_client = self.create_client(
            GetWaypoints,
            '/waypoint_manager/get_waypoints'
        )
        
        # Publishers
        self.actuator_cmd_pub = self.create_publisher(
            String,
            '/actuator/command',
            10
        )
        
        self.actuator_speed_pub = self.create_publisher(
            Int32,
            '/actuator/speed',
            10
        )
        
        # Status timer
        self.status_timer = self.create_timer(5.0, self.print_status)
        
        # State machine timer
        self.state_timer = self.create_timer(0.5, self.state_machine_update)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Inventory Mission Manager Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Scan duration: {self.scan_duration}s per rack')
        self.get_logger().info(f'Actuator speed: {self.actuator_speed} PWM')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting for mission start...')
        self.get_logger().info('Call: ros2 service call /inventory_mission_manager/start_mission std_srvs/srv/Trigger')
        self.get_logger().info('=' * 60)
        
        # Service to start mission
        self.create_service(
            from std_srvs.srv import Trigger,
            '~/start_mission',
            self.start_mission_callback
        )
        
        # Service to stop mission
        self.create_service(
            from std_srvs.srv import Trigger,
            '~/stop_mission',
            self.stop_mission_callback
        )
    
    def start_mission_callback(self, request, response):
        """Start the inventory mission"""
        if self.current_state != self.STATE_IDLE:
            response.success = False
            response.message = f"Mission already running (state: {self.current_state})"
            return response
        
        # Load waypoints
        if not self.load_waypoints():
            response.success = False
            response.message = "Failed to load waypoints"
            return response
        
        if len(self.waypoints) == 0:
            response.success = False
            response.message = "No waypoints found. Use joystick to save waypoints first."
            return response
        
        # Filter for rack waypoints
        self.waypoints = [wp for wp in self.waypoints if wp['type'] == 'rack']
        
        if len(self.waypoints) == 0:
            response.success = False
            response.message = "No RACK waypoints found"
            return response
        
        # Start mission
        self.current_waypoint_index = 0
        self.mission_start_time = time.time()
        self.change_state(self.STATE_NAVIGATING)
        
        response.success = True
        response.message = f"Mission started with {len(self.waypoints)} rack waypoints"
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'🚀 MISSION STARTED - {len(self.waypoints)} racks to scan')
        self.get_logger().info('=' * 60)
        
        return response
    
    def stop_mission_callback(self, request, response):
        """Stop the current mission"""
        if self.current_state == self.STATE_IDLE:
            response.success = False
            response.message = "No mission running"
            return response
        
        self.change_state(self.STATE_IDLE)
        self.send_actuator_command('stop')
        
        response.success = True
        response.message = "Mission stopped"
        
        self.get_logger().warn('⚠️  MISSION STOPPED')
        
        return response
    
    def load_waypoints(self):
        """Load waypoints from waypoint manager"""
        if not self.waypoint_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Waypoint service not available')
            return False
        
        request = GetWaypoints.Request()
        request.waypoint_name = ""  # Get all waypoints
        
        try:
            future = self.waypoint_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            response = future.result()
            if response.success:
                self.waypoints = []
                for i in range(len(response.waypoint_names)):
                    self.waypoints.append({
                        'name': response.waypoint_names[i],
                        'x': response.x_positions[i],
                        'y': response.y_positions[i],
                        'yaw': response.yaw_orientations[i],
                        'type': response.waypoint_types[i]
                    })
                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
                return True
            else:
                self.get_logger().error(f"Failed to get waypoints: {response.message}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")
            return False
    
    def change_state(self, new_state):
        """Change mission state"""
        self.get_logger().info(f"State: {self.current_state} -> {new_state}")
        self.current_state = new_state
        self.state_start_time = time.time()
    
    def state_machine_update(self):
        """Main state machine"""
        if self.current_state == self.STATE_IDLE:
            pass  # Waiting for start command
        
        elif self.current_state == self.STATE_NAVIGATING:
            self.handle_navigating_state()
        
        elif self.current_state == self.STATE_POSITIONING:
            self.handle_positioning_state()
        
        elif self.current_state == self.STATE_LIFTING:
            self.handle_lifting_state()
        
        elif self.current_state == self.STATE_SCANNING:
            self.handle_scanning_state()
        
        elif self.current_state == self.STATE_LOWERING:
            self.handle_lowering_state()
        
        elif self.current_state == self.STATE_COMPLETED:
            pass  # Mission done
    
    def handle_navigating_state(self):
        """Navigate to current waypoint"""
        if not hasattr(self, '_nav_goal_sent'):
            # Send navigation goal
            waypoint = self.waypoints[self.current_waypoint_index]
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = waypoint['x']
            goal_msg.pose.pose.position.y = waypoint['y']
            
            # Convert yaw to quaternion
            q = quaternion_from_euler(0, 0, waypoint['yaw'])
            goal_msg.pose.pose.orientation.x = q[0]
            goal_msg.pose.pose.orientation.y = q[1]
            goal_msg.pose.pose.orientation.z = q[2]
            goal_msg.pose.pose.orientation.w = q[3]
            
            self.get_logger().info(
                f"🎯 Navigating to {waypoint['name']} "
                f"({waypoint['x']:.2f}, {waypoint['y']:.2f})"
            )
            
            self._nav_future = self.nav_client.send_goal_async(goal_msg)
            self._nav_future.add_done_callback(self.nav_goal_response_callback)
            self._nav_goal_sent = True
    
    def nav_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.change_state(self.STATE_FAILED)
            return
        
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(self.nav_result_callback)
    
    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        delattr(self, '_nav_goal_sent')
        
        waypoint = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f"✓ Arrived at {waypoint['name']}")
        
        # Move to next state
        self.change_state(self.STATE_POSITIONING)
    
    def handle_positioning_state(self):
        """Brief pause after arrival"""
        elapsed = time.time() - self.state_start_time
        if elapsed > 2.0:  # 2 second pause
            self.change_state(self.STATE_LIFTING)
    
    def handle_lifting_state(self):
        """Lift actuator"""
        elapsed = time.time() - self.state_start_time
        
        if elapsed < 0.5:
            # Just entered, send lift command
            self.send_actuator_command('up')
        elif elapsed > self.lift_duration:
            # Lifting complete
            self.send_actuator_command('stop')
            self.change_state(self.STATE_SCANNING)
            self.get_logger().info('📸 Starting QR scan...')
    
    def handle_scanning_state(self):
        """Wait for QR scanning"""
        elapsed = time.time() - self.state_start_time
        
        if elapsed > self.scan_duration:
            # Scanning complete
            waypoint = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(f"✓ Scan complete for {waypoint['name']}")
            self.change_state(self.STATE_LOWERING)
    
    def handle_lowering_state(self):
        """Lower actuator"""
        elapsed = time.time() - self.state_start_time
        
        if elapsed < 0.5:
            # Just entered, send lower command
            self.send_actuator_command('down')
        elif elapsed > self.lower_duration:
            # Lowering complete
            self.send_actuator_command('stop')
            
            # Move to next waypoint
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index >= len(self.waypoints):
                # Mission complete!
                self.change_state(self.STATE_COMPLETED)
                self.print_mission_summary()
            else:
                # Next waypoint
                self.change_state(self.STATE_NAVIGATING)
    
    def send_actuator_command(self, command):
        """Send command to actuator"""
        # Set speed
        speed_msg = Int32()
        speed_msg.data = self.actuator_speed
        self.actuator_speed_pub.publish(speed_msg)
        
        # Send command
        cmd_msg = String()
        cmd_msg.data = command
        self.actuator_cmd_pub.publish(cmd_msg)
        
        self.get_logger().info(f"🔧 Actuator: {command}")
    
    def print_status(self):
        """Print current mission status"""
        if self.current_state == self.STATE_IDLE:
            return
        
        elapsed = time.time() - self.mission_start_time if self.mission_start_time else 0
        progress = f"{self.current_waypoint_index + 1}/{len(self.waypoints)}"
        
        self.get_logger().info(
            f"📊 Status: {self.current_state} | "
            f"Progress: {progress} | "
            f"Elapsed: {elapsed/60:.1f} min"
        )
    
    def print_mission_summary(self):
        """Print mission completion summary"""
        elapsed = time.time() - self.mission_start_time
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('  🎉 MISSION COMPLETED!')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Racks scanned: {len(self.waypoints)}')
        self.get_logger().info(f'Total time: {elapsed/60:.1f} minutes')
        self.get_logger().info(f'Average time per rack: {elapsed/len(self.waypoints):.1f} seconds')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    
    # Import here to avoid issues
    from std_srvs.srv import Trigger
    
    node = None
    try:
        node = InventoryMissionManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
