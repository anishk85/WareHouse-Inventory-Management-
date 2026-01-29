#!/usr/bin/env python3
"""
Inventory Node - MongoDB Version
ROS2 node that subscribes to QR detections and stores them in MongoDB Atlas
"""

import rclpy
from rclpy.node import Node
from warehouse_rover_msgs.msg import QRDetectionArray
from datetime import datetime
import os

from warehouse_rover_database.mongodb_manager import MongoDBManager


class InventoryNode(Node):
    """
    ROS2 Node for warehouse inventory management using MongoDB Atlas
    
    Subscribes to:
        /qr_detections (QRDetectionArray): QR code detections from camera
    
    Parameters:
        mongodb_connection_string: MongoDB Atlas connection string
        database_name: Name of MongoDB database
        auto_export: Auto-export mission data on shutdown
        export_dir: Directory for JSON exports
        use_local_mongodb: Use local MongoDB instead of Atlas
    """
    
    def __init__(self):
        super().__init__('inventory_node')
        
        # Declare parameters
        self.declare_parameter('mongodb_connection_string', '')
        self.declare_parameter('database_name', 'warehouse_rover')
        self.declare_parameter('auto_export', True)
        self.declare_parameter('export_dir', '/tmp/inventory_exports')
        self.declare_parameter('use_local_mongodb', False)
        
        # Get parameters
        connection_string = self.get_parameter('mongodb_connection_string').value
        database_name = self.get_parameter('database_name').value
        self.auto_export = self.get_parameter('auto_export').value
        self.export_dir = self.get_parameter('export_dir').value
        use_local = self.get_parameter('use_local_mongodb').value
        
        # If connection string is empty, try environment variable
        if not connection_string:
            connection_string = os.getenv('MONGODB_CONNECTION_STRING')
        
        # Initialize MongoDB
        try:
            self.db = MongoDBManager(
                connection_string=connection_string,
                database_name=database_name,
                use_local=use_local
            )
            self.get_logger().info('✓ MongoDB connected')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MongoDB: {e}')
            self.get_logger().error('Set MONGODB_CONNECTION_STRING environment variable')
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
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Inventory Node Started (MongoDB)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Database: {database_name}')
        self.get_logger().info(f'Mission ID: {self.current_mission_id}')
        self.get_logger().info(f'Auto Export: {self.auto_export}')
        self.get_logger().info(f'Export Directory: {self.export_dir}')
        self.get_logger().info('=' * 60)
    
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
            
            # Store valid detection in MongoDB
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
                    f'✓ [{self.valid_detection_count}] Stored: '
                    f'[{detection.rack_id}][{detection.shelf_id}] '
                    f'{detection.item_code} '
                    f'(conf: {detection.confidence:.2f}, id: {detection_id[:8]}...)'
                )
            else:
                self.get_logger().error(f'❌ Failed to store detection')
    
    def print_status(self):
        """Print current status and statistics"""
        stats = self.db.get_statistics(self.current_mission_id)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Mission Status: {self.current_mission_id}')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Total Detections Received: {self.detection_count}')
        self.get_logger().info(f'Valid Detections Stored:   {self.valid_detection_count}')
        self.get_logger().info(f'Invalid Detections:        {self.invalid_detection_count}')
        self.get_logger().info(f'Unique Racks:              {stats.get("unique_racks", 0)}')
        self.get_logger().info(f'Unique Items:              {stats.get("unique_items", 0)}')
        self.get_logger().info(f'Average Confidence:        {stats.get("average_confidence", 0):.3f}')
        self.get_logger().info('=' * 60)
    
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
