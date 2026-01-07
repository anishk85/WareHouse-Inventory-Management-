#!/usr/bin/env python3
"""
Inventory Node - Neon PostgreSQL Version
ROS2 node that subscribes to QR detections and stores them in Neon database
"""

import rclpy
from rclpy.node import Node
from warehouse_rover_msgs.msg import QRDetectionArray
from datetime import datetime
import os

from warehouse_rover_database.neon_manager import NeonManager


class InventoryNode(Node):
    """
    ROS2 Node for warehouse inventory management using Neon PostgreSQL
    
    Subscribes to:
        /qr_detections (QRDetectionArray): QR code detections from camera
    
    Parameters:
        neon_connection_string: Neon PostgreSQL connection string
        auto_export: Auto-export mission data on shutdown
        export_dir: Directory for JSON exports
        min_connections: Minimum connection pool size
        max_connections: Maximum connection pool size
    """
    
    def __init__(self):
        super().__init__('inventory_node')
        
        # Declare parameters
        self.declare_parameter('neon_connection_string', '')
        self.declare_parameter('auto_export', True)
        self.declare_parameter('export_dir', '/tmp/inventory_exports')
        self.declare_parameter('min_connections', 1)
        self.declare_parameter('max_connections', 5)
        
        # Get parameters
        connection_string = self.get_parameter('neon_connection_string').value
        self.auto_export = self.get_parameter('auto_export').value
        self.export_dir = self.get_parameter('export_dir').value
        min_conn = self.get_parameter('min_connections').value
        max_conn = self.get_parameter('max_connections').value
        
        # If connection string is empty, try environment variable
        if not connection_string:
            connection_string = os.getenv('NEON_CONNECTION_STRING')
        
        # Initialize Neon
        self.get_logger().info('🔄 Connecting to Neon PostgreSQL...')
        try:
            if not connection_string:
                self.get_logger().error('❌ No connection string provided!')
                self.get_logger().error('   Set NEON_CONNECTION_STRING environment variable')
                self.get_logger().error('   Format: postgresql://user:pass@ep-xxx.region.aws.neon.tech/dbname?sslmode=require')
                raise ValueError("Neon connection string not provided")
            
            self.get_logger().info(f'🔌 Connecting to: {connection_string[:50]}...')
            self.db = NeonManager(
                connection_string=connection_string,
                min_connections=min_conn,
                max_connections=max_conn
            )
            self.get_logger().info('✅ Neon PostgreSQL connected successfully')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to connect to Neon: {e}')
            self.get_logger().error('💡 Troubleshooting:')
            self.get_logger().error('   1. Check your connection string in config/neon_params.yaml')
            self.get_logger().error('   2. Verify database exists at https://console.neon.tech')
            self.get_logger().error('   3. Check network connectivity')
            self.get_logger().error('   4. Ensure SSL/TLS certificates are installed')
            raise
        
        # Start new mission
        self.current_mission_id = self._generate_mission_id()
        self.db.start_mission(self.current_mission_id)
        
        # Statistics
        self.detection_count = 0
        self.valid_detection_count = 0
        self.invalid_detection_count = 0
        
        # Create subscription
        self.qr_sub = self.create_subscription(
            QRDetectionArray,
            '/qr_detections',
            self.qr_callback,
            10
        )
        
        # Status timer (print stats every 30 seconds)
        self.status_timer = self.create_timer(30.0, self.print_status)
        
        self.get_logger().info('╔' + '═' * 58 + '╗')
        self.get_logger().info('║  🚀 Inventory Node Started (Neon PostgreSQL)           ║')
        self.get_logger().info('╠' + '═' * 58 + '╣')
        self.get_logger().info(f'║  📋 Mission ID: {self.current_mission_id:<40} ║')
        self.get_logger().info(f'║  💾 Auto Export: {str(self.auto_export):<38} ║')
        self.get_logger().info(f'║  📁 Export Dir: {self.export_dir[:38]:<38} ║')
        self.get_logger().info(f'║  🔗 Min/Max Connections: {min_conn}/{max_conn:<30} ║')
        self.get_logger().info('╚' + '═' * 58 + '╝')
    
    def _generate_mission_id(self) -> str:
        """Generate a unique mission ID based on timestamp"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return f'MISSION_{timestamp}'
    
    def qr_callback(self, msg: QRDetectionArray):
        """
        Callback for QR detection messages
        
        Args:
            msg: QRDetectionArray message
        """
        if not msg.detections:
            return
        
        for detection in msg.detections:
            self.detection_count += 1
            
            # Check if detection is valid
            if not detection.is_valid:
                self.invalid_detection_count += 1
                self.get_logger().warn(
                    f'⚠️  Invalid QR [{self.detection_count}]: "{detection.qr_data}" - '
                    f'{detection.error_message}'
                )
                continue
            
            # Store valid detection in Neon
            detection_id = self.db.add_detection(
                mission_id=self.current_mission_id,
                rack_id=detection.rack_id,
                shelf_id=detection.shelf_id,
                item_code=detection.item_code,
                confidence=detection.confidence,
                qr_data=detection.qr_data,
                additional_data={
                    'center_x': detection.center.x,
                    'center_y': detection.center.y,
                    'size_pixels': detection.size_pixels,
                    'processing_time_ms': msg.processing_time_ms,
                    'camera_height': msg.camera_height
                }
            )
            
            if detection_id:
                self.valid_detection_count += 1
                self.get_logger().info(
                    f'✅ [#{self.valid_detection_count:04d}] 📦 Stored: '
                    f'🏷️ [{detection.rack_id}][{detection.shelf_id}] '
                    f'📋 {detection.item_code} '
                    f'📊 (conf: {detection.confidence:.2f}, 🆔 id: {detection_id})'
                )
            else:
                self.get_logger().error(f'❌ Failed to store detection #{self.detection_count}')
    
    def print_status(self):
        """Print current status and statistics"""
        stats = self.db.get_statistics(self.current_mission_id)
        
        self.get_logger().info('┏' + '━' * 58 + '┓')
        self.get_logger().info(f'┃  📊 Mission Status: {self.current_mission_id:<35} ┃')
        self.get_logger().info('┣' + '━' * 58 + '┫')
        self.get_logger().info(f'┃  📥 Total Detections Received: {self.detection_count:<25} ┃')
        self.get_logger().info(f'┃  ✅ Valid Detections Stored:   {self.valid_detection_count:<25} ┃')
        self.get_logger().info(f'┃  ❌ Invalid Detections:        {self.invalid_detection_count:<25} ┃')
        self.get_logger().info(f'┃  🏷️  Unique Racks:              {stats.get("unique_racks", 0):<25} ┃')
        self.get_logger().info(f'┃  📦 Unique Items:              {stats.get("unique_items", 0):<25} ┃')
        self.get_logger().info(f'┃  📊 Average Confidence:        {stats.get("average_confidence", 0):>6.3f}                    ┃')
        self.get_logger().info('┗' + '━' * 58 + '┛')
    
    def shutdown(self):
        """Cleanup on node shutdown"""
        self.get_logger().info('Shutting down Inventory Node...')
        
        # End mission
        self.db.end_mission(self.current_mission_id)
        
        # Export if enabled
        if self.auto_export:
            export_path = os.path.join(
                self.export_dir,
                f'{self.current_mission_id}.json'
            )
            self.get_logger().info(f'Exporting mission data to: {export_path}')
            self.db.export_to_json(self.current_mission_id, export_path)
        
        # Print final stats
        self.print_status()
        
        # Close database connection
        self.db.close()
        
        self.get_logger().info('✓ Shutdown complete')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = None
    try:
        node = InventoryNode()
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
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()