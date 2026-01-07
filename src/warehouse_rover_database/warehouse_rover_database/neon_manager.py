#!/usr/bin/env python3
"""
Neon PostgreSQL Manager for Warehouse Rover Database
Replaces MongoDB with Neon (serverless PostgreSQL)
"""

import psycopg2
from psycopg2.extras import RealDictCursor
from psycopg2 import pool
from datetime import datetime
from typing import List, Dict, Optional
import json
import os


class NeonManager:
    """
    Neon PostgreSQL Manager for storing warehouse QR detections
    
    Features:
    - Cloud-based storage (Neon serverless Postgres)
    - Mission management (start/end missions)
    - Detection storage (QR code data)
    - Query and export capabilities
    """
    
    def __init__(
        self,
        connection_string: str = None,
        min_connections: int = 1,
        max_connections: int = 5
    ):
        """
        Initialize Neon PostgreSQL connection
        
        Args:
            connection_string: Neon connection string
                Format: postgresql://username:password@ep-xxx.region.aws.neon.tech/dbname?sslmode=require
            min_connections: Minimum connections in pool
            max_connections: Maximum connections in pool
        """
        # Get connection string from parameter or environment variable
        if connection_string is None:
            connection_string = os.getenv('NEON_CONNECTION_STRING')
        
        if connection_string is None:
            raise ValueError(
                "Neon connection string not provided. "
                "Set NEON_CONNECTION_STRING environment variable or pass connection_string parameter."
            )
        
        self.connection_string = connection_string
        self.connection_pool = None
        
        self._connect(min_connections, max_connections)
        self._create_tables()
    
    def _connect(self, min_conn: int, max_conn: int):
        """Establish connection pool to Neon"""
        try:
            self.connection_pool = pool.SimpleConnectionPool(
                min_conn,
                max_conn,
                self.connection_string
            )
            
            # Test connection
            conn = self.connection_pool.getconn()
            cursor = conn.cursor()
            cursor.execute("SELECT version();")
            version = cursor.fetchone()
            cursor.close()
            self.connection_pool.putconn(conn)
            
            print(f"✓ Connected to Neon PostgreSQL")
            print(f"  Version: {version[0][:50]}...")
            
        except Exception as e:
            print(f"❌ Failed to connect to Neon: {e}")
            raise
    
    def _create_tables(self):
        """Create database tables if they don't exist"""
        conn = self.connection_pool.getconn()
        try:
            cursor = conn.cursor()
            
            # Missions table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS missions (
                    id SERIAL PRIMARY KEY,
                    mission_id VARCHAR(255) UNIQUE NOT NULL,
                    start_time TIMESTAMP NOT NULL,
                    end_time TIMESTAMP,
                    status VARCHAR(50) DEFAULT 'in_progress',
                    total_detections INTEGER DEFAULT 0,
                    metadata JSONB,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)
            
            # Detections table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS detections (
                    id SERIAL PRIMARY KEY,
                    mission_id VARCHAR(255) NOT NULL,
                    rack_id VARCHAR(50) NOT NULL,
                    shelf_id VARCHAR(50) NOT NULL,
                    item_code VARCHAR(100) NOT NULL,
                    qr_data TEXT,
                    confidence REAL NOT NULL,
                    timestamp TIMESTAMP NOT NULL,
                    metadata JSONB,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (mission_id) REFERENCES missions(mission_id) ON DELETE CASCADE
                )
            """)
            
            # Create indexes for better performance
            cursor.execute("""
                CREATE INDEX IF NOT EXISTS idx_detections_mission 
                ON detections(mission_id, timestamp DESC)
            """)
            
            cursor.execute("""
                CREATE INDEX IF NOT EXISTS idx_detections_rack 
                ON detections(rack_id)
            """)
            
            cursor.execute("""
                CREATE INDEX IF NOT EXISTS idx_detections_item 
                ON detections(item_code)
            """)
            
            conn.commit()
            cursor.close()
            print("✓ Database tables created/verified")
            
        except Exception as e:
            conn.rollback()
            print(f"❌ Failed to create tables: {e}")
            raise
        finally:
            self.connection_pool.putconn(conn)
    
    def start_mission(self, mission_id: str) -> bool:
        """
        Start a new mission
        
        Args:
            mission_id: Unique mission identifier
            
        Returns:
            True if successful, False otherwise
        """
        conn = self.connection_pool.getconn()
        try:
            cursor = conn.cursor()
            
            metadata = json.dumps({
                "created_by": "warehouse_rover",
                "version": "1.0"
            })
            
            cursor.execute("""
                INSERT INTO missions (mission_id, start_time, status, metadata)
                VALUES (%s, %s, %s, %s)
            """, (mission_id, datetime.utcnow(), 'in_progress', metadata))
            
            conn.commit()
            cursor.close()
            print(f"✓ Mission started: {mission_id}")
            return True
            
        except Exception as e:
            conn.rollback()
            print(f"❌ Failed to start mission: {e}")
            return False
        finally:
            self.connection_pool.putconn(conn)
    
    def end_mission(self, mission_id: str) -> bool:
        """
        End an active mission
        
        Args:
            mission_id: Mission identifier to end
            
        Returns:
            True if successful, False otherwise
        """
        conn = self.connection_pool.getconn()
        try:
            cursor = conn.cursor()
            
            # Count total detections
            cursor.execute("""
                SELECT COUNT(*) FROM detections WHERE mission_id = %s
            """, (mission_id,))
            detection_count = cursor.fetchone()[0]
            
            # Update mission
            cursor.execute("""
                UPDATE missions
                SET end_time = %s, status = %s, total_detections = %s
                WHERE mission_id = %s
            """, (datetime.utcnow(), 'completed', detection_count, mission_id))
            
            conn.commit()
            
            if cursor.rowcount > 0:
                cursor.close()
                print(f"✓ Mission ended: {mission_id} ({detection_count} detections)")
                return True
            else:
                cursor.close()
                print(f"⚠️  Mission not found: {mission_id}")
                return False
                
        except Exception as e:
            conn.rollback()
            print(f"❌ Failed to end mission: {e}")
            return False
        finally:
            self.connection_pool.putconn(conn)
    
    def add_detection(
        self,
        mission_id: str,
        rack_id: str,
        shelf_id: str,
        item_code: str,
        confidence: float,
        qr_data: str = "",
        additional_data: Dict = None
    ) -> Optional[int]:
        """
        Add a QR detection to the database
        
        Args:
            mission_id: Mission this detection belongs to
            rack_id: Rack identifier (e.g., "R01")
            shelf_id: Shelf identifier (e.g., "S1")
            item_code: Item code (e.g., "ITEM001")
            confidence: Detection confidence (0.0 to 1.0)
            qr_data: Raw QR code data
            additional_data: Any additional metadata
            
        Returns:
            Detection ID (int) if successful, None otherwise
        """
        conn = self.connection_pool.getconn()
        try:
            cursor = conn.cursor()
            
            metadata = json.dumps(additional_data) if additional_data else None
            
            cursor.execute("""
                INSERT INTO detections 
                (mission_id, rack_id, shelf_id, item_code, qr_data, confidence, timestamp, metadata)
                VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
                RETURNING id
            """, (mission_id, rack_id, shelf_id, item_code, qr_data, confidence, 
                  datetime.utcnow(), metadata))
            
            detection_id = cursor.fetchone()[0]
            conn.commit()
            cursor.close()
            return detection_id
            
        except Exception as e:
            conn.rollback()
            print(f"❌ Failed to add detection: {e}")
            return None
        finally:
            self.connection_pool.putconn(conn)
    
    def get_detections_by_mission(self, mission_id: str) -> List[Dict]:
        """
        Get all detections for a mission
        
        Args:
            mission_id: Mission identifier
            
        Returns:
            List of detection dictionaries
        """
        conn = self.connection_pool.getconn()
        try:
            cursor = conn.cursor(cursor_factory=RealDictCursor)
            
            cursor.execute("""
                SELECT * FROM detections
                WHERE mission_id = %s
                ORDER BY timestamp ASC
            """, (mission_id,))
            
            detections = cursor.fetchall()
            cursor.close()
            
            # Convert to list of dicts and format timestamps
            result = []
            for det in detections:
                det_dict = dict(det)
                if 'timestamp' in det_dict:
                    det_dict['timestamp'] = det_dict['timestamp'].isoformat()
                if 'created_at' in det_dict:
                    det_dict['created_at'] = det_dict['created_at'].isoformat()
                result.append(det_dict)
            
            return result
            
        except Exception as e:
            print(f"❌ Failed to get detections: {e}")
            return []
        finally:
            self.connection_pool.putconn(conn)
    
    def get_detections_by_rack(self, rack_id: str) -> List[Dict]:
        """
        Get all detections for a specific rack
        
        Args:
            rack_id: Rack identifier
            
        Returns:
            List of detection dictionaries
        """
        conn = self.connection_pool.getconn()
        try:
            cursor = conn.cursor(cursor_factory=RealDictCursor)
            
            cursor.execute("""
                SELECT * FROM detections
                WHERE rack_id = %s
                ORDER BY timestamp DESC
            """, (rack_id,))
            
            detections = cursor.fetchall()
            cursor.close()
            
            result = []
            for det in detections:
                det_dict = dict(det)
                if 'timestamp' in det_dict:
                    det_dict['timestamp'] = det_dict['timestamp'].isoformat()
                if 'created_at' in det_dict:
                    det_dict['created_at'] = det_dict['created_at'].isoformat()
                result.append(det_dict)
            
            return result
            
        except Exception as e:
            print(f"❌ Failed to get detections by rack: {e}")
            return []
        finally:
            self.connection_pool.putconn(conn)
    
    def get_mission_summary(self, mission_id: str) -> Optional[Dict]:
        """
        Get mission summary
        
        Args:
            mission_id: Mission identifier
            
        Returns:
            Mission dictionary or None
        """
        conn = self.connection_pool.getconn()
        try:
            cursor = conn.cursor(cursor_factory=RealDictCursor)
            
            cursor.execute("""
                SELECT * FROM missions WHERE mission_id = %s
            """, (mission_id,))
            
            mission = cursor.fetchone()
            cursor.close()
            
            if mission:
                mission_dict = dict(mission)
                if 'start_time' in mission_dict:
                    mission_dict['start_time'] = mission_dict['start_time'].isoformat()
                if 'end_time' in mission_dict and mission_dict['end_time']:
                    mission_dict['end_time'] = mission_dict['end_time'].isoformat()
                if 'created_at' in mission_dict:
                    mission_dict['created_at'] = mission_dict['created_at'].isoformat()
                return mission_dict
            
            return None
            
        except Exception as e:
            print(f"❌ Failed to get mission summary: {e}")
            return None
        finally:
            self.connection_pool.putconn(conn)
    
    def export_to_json(self, mission_id: str, output_path: str) -> bool:
        """
        Export mission data to JSON file
        
        Args:
            mission_id: Mission to export
            output_path: Output file path
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Get mission summary
            summary = self.get_mission_summary(mission_id)
            if not summary:
                print(f"⚠️  Mission not found: {mission_id}")
                return False
            
            # Get detections
            detections = self.get_detections_by_mission(mission_id)
            
            # Combine data
            export_data = {
                "mission": summary,
                "detections": detections,
                "export_time": datetime.utcnow().isoformat(),
                "total_detections": len(detections)
            }
            
            # Write to file
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            with open(output_path, 'w') as f:
                json.dump(export_data, f, indent=2)
            
            print(f"✓ Exported to: {output_path}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to export: {e}")
            return False
    
    def get_statistics(self, mission_id: str = None) -> Dict:
        """
        Get statistics for a mission or overall
        
        Args:
            mission_id: Mission identifier (None for overall stats)
            
        Returns:
            Dictionary with statistics
        """
        conn = self.connection_pool.getconn()
        try:
            cursor = conn.cursor()
            
            if mission_id:
                cursor.execute("""
                    SELECT 
                        COUNT(*) as total_detections,
                        AVG(confidence) as avg_confidence,
                        COUNT(DISTINCT rack_id) as unique_racks,
                        COUNT(DISTINCT item_code) as unique_items
                    FROM detections
                    WHERE mission_id = %s
                """, (mission_id,))
            else:
                cursor.execute("""
                    SELECT 
                        COUNT(*) as total_detections,
                        AVG(confidence) as avg_confidence,
                        COUNT(DISTINCT rack_id) as unique_racks,
                        COUNT(DISTINCT item_code) as unique_items
                    FROM detections
                """)
            
            result = cursor.fetchone()
            cursor.close()
            
            return {
                "total_detections": result[0] or 0,
                "average_confidence": round(result[1], 3) if result[1] else 0.0,
                "unique_racks": result[2] or 0,
                "unique_items": result[3] or 0
            }
                
        except Exception as e:
            print(f"❌ Failed to get statistics: {e}")
            return {
                "total_detections": 0,
                "average_confidence": 0.0,
                "unique_racks": 0,
                "unique_items": 0
            }
        finally:
            self.connection_pool.putconn(conn)
    
    def close(self):
        """Close all connections in pool"""
        if self.connection_pool:
            self.connection_pool.closeall()
            print("✓ Neon connection pool closed")


# Test function
if __name__ == "__main__":
    import sys
    
    print("=" * 60)
    print("  Neon PostgreSQL Manager Test")
    print("=" * 60)
    print()
    
    # Test with Neon
    try:
        # You need to set NEON_CONNECTION_STRING environment variable
        # or pass it directly: db = NeonManager(connection_string="postgresql://...")
        db = NeonManager()
        
        # Test mission
        test_mission = "TEST_MISSION_001"
        
        print("\n1. Starting test mission...")
        db.start_mission(test_mission)
        
        print("\n2. Adding test detections...")
        det1 = db.add_detection(test_mission, "R01", "S1", "ITEM001", 0.95, "R01-S1-ITEM001")
        det2 = db.add_detection(test_mission, "R01", "S2", "ITEM002", 0.88, "R01-S2-ITEM002")
        det3 = db.add_detection(test_mission, "R02", "S1", "ITEM003", 0.92, "R02-S1-ITEM003")
        print(f"   Detection IDs: {det1}, {det2}, {det3}")
        
        print("\n3. Getting statistics...")
        stats = db.get_statistics(test_mission)
        print(f"   Total detections: {stats['total_detections']}")
        print(f"   Average confidence: {stats['average_confidence']}")
        print(f"   Unique racks: {stats['unique_racks']}")
        
        print("\n4. Ending mission...")
        db.end_mission(test_mission)
        
        print("\n5. Exporting to JSON...")
        db.export_to_json(test_mission, "/tmp/test_export.json")
        
        print("\n6. Getting mission summary...")
        summary = db.get_mission_summary(test_mission)
        print(f"   Status: {summary['status']}")
        print(f"   Total detections: {summary['total_detections']}")
        
        db.close()
        
        print("\n" + "=" * 60)
        print("✓ All tests passed!")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        print("\nMake sure to set NEON_CONNECTION_STRING environment variable:")
        print("export NEON_CONNECTION_STRING='postgresql://user:pass@ep-xxx.region.aws.neon.tech/dbname?sslmode=require'")
        sys.exit(1)