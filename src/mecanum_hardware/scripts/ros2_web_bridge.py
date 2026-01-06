#!/usr/bin/env python3
"""
ROS2 WebSocket Bridge Server

This server bridges ROS2 topics/services/actions to WebSocket connections
for the Next.js frontend. It allows the web GUI to:
- Subscribe to ROS2 topics (sensor data, status, etc.)
- Publish to ROS2 topics (commands)
- Call ROS2 services (waypoint management)
- Monitor ROS2 actions (navigation progress)
- Launch ROS2 nodes
"""

import asyncio
import json
import subprocess
import os
import signal
from typing import Dict, Set
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import LaserScan, Imu, Image
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from warehouse_rover_msgs.msg import QRDetectionArray

try:
    import websockets
    import websockets.server
except ImportError:
    print("Installing websockets...")
    subprocess.check_call(["pip3", "install", "websockets"])
    import websockets
    import websockets.server


class ROS2WebBridge(Node):
    """
    Bridge between ROS2 and WebSocket clients
    """
    
    def __init__(self):
        super().__init__('ros2_web_bridge')
        
        # WebSocket clients
        self.websocket_clients: Set = set()
        
        # Event loop reference for thread-safe async calls
        self.loop = None
        
        # Launch process tracking
        self.launch_processes: Dict[str, subprocess.Popen] = {}
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # ============= SUBSCRIBERS =============
        
        # Status topics
        self.create_subscription(String, '/waypoint_follower/status', 
                                self.make_topic_callback('waypoint_status'), 10)
        self.create_subscription(String, '/inventory_mission/status',
                                self.make_topic_callback('mission_status'), 10)
        self.create_subscription(String, '/actuator/status',
                                self.make_topic_callback('actuator_status'), 10)
        
        # Sensor topics
        self.create_subscription(LaserScan, '/scan',
                                self.lidar_callback, self.sensor_qos)
        self.create_subscription(Imu, '/imu/data',
                                self.imu_callback, self.sensor_qos)
        self.create_subscription(Odometry, '/mecanum_drive_controller/odometry',
                                self.odom_callback, 10)
        
        # QR Detection
        self.create_subscription(QRDetectionArray, '/qr_detections',
                                self.qr_callback, 10)
        
        # Navigation topics
        self.create_subscription(PoseStamped, '/goal_pose',
                                self.make_topic_callback('goal_pose'), 10)
        self.create_subscription(Path, '/plan',
                                self.path_callback, 10)
        
        # Laser distance sensor
        self.create_subscription(Float32, '/laser/distance',
                                self.make_topic_callback('laser_distance'), 10)
        
        # ============= PUBLISHERS =============
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/mecanum_drive_controller/cmd_vel_unstamped', 10)
        self.actuator_cmd_pub = self.create_publisher(String, '/actuator/command', 10)
        self.qr_enable_pub = self.create_publisher(Bool, '/qr_enable', 10)
        
        # ============= TIMERS =============
        
        # Note: System status will be sent on-demand via WebSocket handler
        # Not using create_timer to avoid executor conflicts
        
        self.get_logger().info('ROS2 Web Bridge initialized')
    
    def make_topic_callback(self, topic_name: str):
        """Factory function to create topic callbacks"""
        def callback(msg):
            data = {
                'type': 'topic',
                'topic': topic_name,
                'data': self.msg_to_dict(msg),
                'timestamp': datetime.now().isoformat()
            }
            self.broadcast_sync(data)
        return callback
    
    def lidar_callback(self, msg: LaserScan):
        """LiDAR data - send summary to reduce bandwidth"""
        data = {
            'type': 'topic',
            'topic': 'lidar',
            'data': {
                'range_min': float(msg.range_min),
                'range_max': float(msg.range_max),
                'angle_min': float(msg.angle_min),
                'angle_max': float(msg.angle_max),
                'num_readings': len(msg.ranges),
                'closest_obstacle': float(min([r for r in msg.ranges if r > 0] or [msg.range_max]))
            },
            'timestamp': datetime.now().isoformat()
        }
        self.broadcast_sync(data)
    
    def imu_callback(self, msg: Imu):
        """IMU data"""
        data = {
            'type': 'topic',
            'topic': 'imu',
            'data': {
                'orientation': {
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z,
                    'w': msg.orientation.w
                },
                'angular_velocity': {
                    'x': msg.angular_velocity.x,
                    'y': msg.angular_velocity.y,
                    'z': msg.angular_velocity.z
                },
                'linear_acceleration': {
                    'x': msg.linear_acceleration.x,
                    'y': msg.linear_acceleration.y,
                    'z': msg.linear_acceleration.z
                }
            },
            'timestamp': datetime.now().isoformat()
        }
        self.broadcast_sync(data)
    
    def odom_callback(self, msg: Odometry):
        """Odometry data"""
        data = {
            'type': 'topic',
            'topic': 'odometry',
            'data': {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                },
                'linear_velocity': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular_velocity': {
                    'x': msg.twist.twist.angular.x,
                    'y': msg.twist.twist.angular.y,
                    'z': msg.twist.twist.angular.z
                }
            },
            'timestamp': datetime.now().isoformat()
        }
        self.broadcast_sync(data)
    
    def qr_callback(self, msg: QRDetectionArray):
        """QR detection data"""
        detections = []
        for det in msg.detections:
            detections.append({
                'qr_data': det.qr_data,
                'rack_id': det.rack_id,
                'shelf_id': det.shelf_id,
                'item_code': det.item_code,
                'confidence': det.confidence,
                'is_valid': det.is_valid,
                'error_message': det.error_message
            })
        
        data = {
            'type': 'topic',
            'topic': 'qr_detections',
            'data': {
                'detection_count': msg.detection_count,
                'detections': detections,
                'processing_time_ms': msg.processing_time_ms
            },
            'timestamp': datetime.now().isoformat()
        }
        self.broadcast_sync(data)
    
    def path_callback(self, msg: Path):
        """Navigation path"""
        data = {
            'type': 'topic',
            'topic': 'nav_path',
            'data': {
                'frame_id': msg.header.frame_id,
                'num_poses': len(msg.poses)
            },
            'timestamp': datetime.now().isoformat()
        }
        self.broadcast_sync(data)
    
    def publish_system_status(self):
        """Publish system status periodically"""
        # Get ROS2 node list
        node_list = self.get_node_names()
        
        # Get topic list
        topic_list = [name for name, _ in self.get_topic_names_and_types()]
        
        data = {
            'type': 'system_status',
            'data': {
                'active_nodes': len(node_list),
                'active_topics': len(topic_list),
                'launch_processes': list(self.launch_processes.keys()),
                'bridge_uptime': self.get_clock().now().nanoseconds / 1e9
            },
            'timestamp': datetime.now().isoformat()
        }
        self.broadcast_sync(data)
    
    def msg_to_dict(self, msg):
        """Convert ROS2 message to dictionary"""
        if isinstance(msg, String):
            return {'data': msg.data}
        elif isinstance(msg, Bool):
            return {'data': msg.data}
        elif isinstance(msg, Float32):
            return {'data': float(msg.data)}
        else:
            # Generic conversion
            return {'raw': str(msg)}
    
    def broadcast_sync(self, data: dict):
        """Thread-safe wrapper for broadcasting from ROS2 callbacks"""
        if self.loop and self.websocket_clients:
            asyncio.run_coroutine_threadsafe(self.broadcast(data), self.loop)
    
    async def broadcast(self, data: dict):
        """Broadcast data to all connected clients"""
        if self.websocket_clients:
            message = json.dumps(data)
            await asyncio.gather(
                *[client.send(message) for client in self.websocket_clients],
                return_exceptions=True
            )
    
    async def handle_client_message(self, websocket, message: str):
        """Handle incoming WebSocket message from client"""
        try:
            data = json.loads(message)
            action = data.get('action')
            
            if action == 'publish_cmd_vel':
                # Publish velocity command
                twist = Twist()
                # Ensure proper float conversion
                try:
                    twist.linear.x = float(data.get('linear_x', 0.0))
                    twist.linear.y = float(data.get('linear_y', 0.0))
                    twist.angular.z = float(data.get('angular_z', 0.0))
                except (ValueError, TypeError) as e:
                    self.get_logger().error(f'Invalid velocity values: {e}')
                    await websocket.send(json.dumps({'status': 'error', 'message': f'Invalid velocity values: {e}'}))
                    return
                
                self.cmd_vel_pub.publish(twist)
                await websocket.send(json.dumps({'status': 'success', 'action': 'publish_cmd_vel'}))
            
            elif action == 'actuator_command':
                # Control actuator
                command = data.get('command', 'stop')
                self.get_logger().info(f'Actuator command received: {command}')
                msg = String()
                msg.data = command
                self.actuator_cmd_pub.publish(msg)
                self.get_logger().info(f'Published to /actuator/command: {command}')
                await websocket.send(json.dumps({'status': 'success', 'action': 'actuator_command', 'command': command}))
            
            elif action == 'qr_enable':
                # Enable/disable QR detection
                msg = Bool()
                msg.data = data.get('enable', False)
                self.qr_enable_pub.publish(msg)
                await websocket.send(json.dumps({'status': 'success', 'action': 'qr_enable'}))
            
            elif action == 'launch':
                # Launch ROS2 launch file
                launch_name = data.get('launch_name')
                launch_file = data.get('launch_file')
                params = data.get('params', {})
                
                result = self.launch_ros2(launch_name, launch_file, params)
                await websocket.send(json.dumps(result))
            
            elif action == 'stop_launch':
                # Stop a running launch
                launch_name = data.get('launch_name')
                result = self.stop_launch(launch_name)
                await websocket.send(json.dumps(result))
            
            elif action == 'get_waypoints':
                # Get saved waypoints
                waypoints = self.get_waypoints()
                await websocket.send(json.dumps({'status': 'success', 'waypoints': waypoints}))
            
            else:
                await websocket.send(json.dumps({'status': 'error', 'message': f'Unknown action: {action}'}))
        
        except Exception as e:
            self.get_logger().error(f'Error handling message: {e}')
            await websocket.send(json.dumps({'status': 'error', 'message': str(e)}))
    
    def launch_ros2(self, launch_name: str, launch_file: str, params: dict):
        """Launch a ROS2 launch file"""
        try:
            # Parse launch file - could be "package launch_file" or just "launch_file"
            parts = launch_file.split()
            if len(parts) == 2:
                cmd = ['ros2', 'launch', parts[0], parts[1]]
            else:
                cmd = ['ros2', 'launch', launch_file]
            
            # Add parameters
            for key, value in params.items():
                cmd.append(f'{key}:={value}')
            
            # Log the command being executed
            cmd_str = ' '.join(cmd)
            self.get_logger().info(f'Executing: {cmd_str}')
            
            # Launch process - use script to force line buffering
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = '0'
            env['PYTHONUNBUFFERED'] = '1'
            
            # Use stdbuf to force line buffering, or script for pseudo-tty
            # This ensures we get output line by line
            full_cmd = ['stdbuf', '-oL', '-eL'] + cmd
            
            process = subprocess.Popen(
                full_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                preexec_fn=os.setsid,
                text=True,
                bufsize=1
            )
            
            self.launch_processes[launch_name] = {
                'process': process,
                'cmd': cmd_str,
                'start_time': datetime.now().isoformat()
            }
            
            # Start a thread to read output and broadcast logs
            import threading
            log_thread = threading.Thread(
                target=self._stream_process_output,
                args=(launch_name, process),
                daemon=True
            )
            log_thread.start()
            
            self.get_logger().info(f'Launched: {launch_name} (PID: {process.pid})')
            
            return {
                'status': 'success',
                'action': 'launch',
                'launch_name': launch_name,
                'pid': process.pid,
                'command': cmd_str
            }
        
        except Exception as e:
            self.get_logger().error(f'Failed to launch {launch_name}: {e}')
            return {
                'status': 'error',
                'action': 'launch',
                'message': str(e)
            }
    
    def _stream_process_output(self, launch_name: str, process):
        """Stream process output to WebSocket clients"""
        try:
            self.get_logger().info(f'Starting log stream for {launch_name}')
            
            while True:
                line = process.stdout.readline()
                if not line and process.poll() is not None:
                    break
                if line:
                    line = line.rstrip()
                    self.get_logger().info(f'[{launch_name}] {line}')
                    
                    # Broadcast log line to all clients
                    log_data = {
                        'type': 'launch_log',
                        'launch_name': launch_name,
                        'log': line,
                        'timestamp': datetime.now().isoformat()
                    }
                    self.broadcast_sync(log_data)
            
            # Process ended
            exit_code = process.poll()
            self.get_logger().info(f'{launch_name} ended with code: {exit_code}')
            
            end_data = {
                'type': 'launch_ended',
                'launch_name': launch_name,
                'exit_code': exit_code,
                'timestamp': datetime.now().isoformat()
            }
            self.broadcast_sync(end_data)
            self.broadcast_sync(end_data)
            
            # Update process status
            if launch_name in self.launch_processes:
                self.launch_processes[launch_name]['exit_code'] = exit_code
                self.launch_processes[launch_name]['ended'] = True
                
        except Exception as e:
            self.get_logger().error(f'Error streaming output for {launch_name}: {e}')
    
    def stop_launch(self, launch_name: str):
        """Stop a running launch"""
        try:
            if launch_name in self.launch_processes:
                proc_info = self.launch_processes[launch_name]
                process = proc_info['process'] if isinstance(proc_info, dict) else proc_info
                
                # Send SIGTERM to process group
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                
                # Wait for termination
                process.wait(timeout=5)
                
                del self.launch_processes[launch_name]
                
                self.get_logger().info(f'Stopped: {launch_name}')
                
                return {
                    'status': 'success',
                    'action': 'stop_launch',
                    'launch_name': launch_name
                }
            else:
                return {
                    'status': 'error',
                    'message': f'Launch {launch_name} not found'
                }
        
        except Exception as e:
            self.get_logger().error(f'Failed to stop {launch_name}: {e}')
            return {
                'status': 'error',
                'message': str(e)
            }
    
    def get_waypoints(self):
        """Read waypoints from file"""
        try:
            import yaml
            with open('/tmp/waypoints.yaml', 'r') as f:
                data = yaml.safe_load(f)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f'Failed to read waypoints: {e}')
            return []


async def websocket_server(bridge: ROS2WebBridge, websocket):
    """Handle WebSocket connections"""
    bridge.websocket_clients.add(websocket)
    bridge.get_logger().info(f'Client connected: {websocket.remote_address}')
    
    try:
        # Send initial connection message
        await websocket.send(json.dumps({
            'type': 'connection',
            'status': 'connected',
            'timestamp': datetime.now().isoformat()
        }))
        
        # Handle incoming messages
        async for message in websocket:
            await bridge.handle_client_message(websocket, message)
    
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        bridge.websocket_clients.remove(websocket)
        bridge.get_logger().info(f'Client disconnected: {websocket.remote_address}')


def main():
    rclpy.init()
    
    bridge = ROS2WebBridge()
    
    # Create executor for ROS2
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    
    # Run ROS2 in separate thread
    import threading
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    # Run WebSocket server
    async def run_server():
        # Set the event loop reference for thread-safe broadcasting
        bridge.loop = asyncio.get_event_loop()
        
        # Create WebSocket server with proper origin handling
        async def handler(websocket):
            await websocket_server(bridge, websocket)
        
        async with websockets.server.serve(
            handler,
            "0.0.0.0",
            9090,
            # Allow all origins for development
            origins=None  # None means allow all origins
        ):
            bridge.get_logger().info('=' * 70)
            bridge.get_logger().info('  ROS2 WebSocket Bridge Server Started')
            bridge.get_logger().info('=' * 70)
            bridge.get_logger().info('WebSocket Server: ws://0.0.0.0:9090')
            bridge.get_logger().info('Clients can now connect from Next.js frontend')
            bridge.get_logger().info('=' * 70)
            await asyncio.Future()  # Run forever
    
    try:
        asyncio.run(run_server())
    except KeyboardInterrupt:
        bridge.get_logger().info('Shutting down...')
    finally:
        # Stop all launch processes
        for name, process in bridge.launch_processes.items():
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except:
                pass
        
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
