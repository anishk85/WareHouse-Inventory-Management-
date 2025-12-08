#!/usr/bin/env python3
"""
MongoDB Atlas Manager for Warehouse Rover Database
Replaces the C++ SQLite implementation with cloud-based MongoDB Atlas
"""

from pymongo import MongoClient
from pymongo.errors import ConnectionFailure, ServerSelectionTimeoutError
from datetime import datetime
from typing import List, Dict, Optional
import json
import os


class MongoDBManager:
    """
    MongoDB Atlas Manager for storing warehouse QR detections
    
    Features:
    - Cloud-based storage (MongoDB Atlas)
    - Mission management (start/end missions)
    - Detection storage (QR code data)
    - Query and export capabilities
    """
    
    def __init__(
        self,
        connection_string: str = None,
        database_name: str = "warehouse_rover",
        use_local: bool = False
    ):
        """
        Initialize MongoDB connection
        
        Args:
            connection_string: MongoDB Atlas connection string
                Format: mongodb+srv://<username>:<password>@<cluster>.mongodb.net/
            database_name: Name of the database
            use_local: If True, connect to local MongoDB instead of Atlas
        """
        # Get connection string from parameter or environment variable
        if connection_string is None:
            connection_string = os.getenv(
                'MONGODB_CONNECTION_STRING',
                'mongodb://localhost:27017/' if use_local else None
            )
        
        if connection_string is None:
            raise ValueError(
                "MongoDB connection string not provided. "
                "Set MONGODB_CONNECTION_STRING environment variable or pass connection_string parameter."
            )
        
        self.connection_string = connection_string
        self.database_name = database_name
        self.client = None
        self.db = None
        self.missions_collection = None
        self.detections_collection = None
        
        self._connect()
    
    def _connect(self):
        """Establish connection to MongoDB"""
        try:
            # Connect with timeout
            self.client = MongoClient(
                self.connection_string,
                serverSelectionTimeoutMS=5000,
                connectTimeoutMS=10000
            )
            
            # Test connection
            self.client.admin.command('ping')
            
            # Get database and collections
            self.db = self.client[self.database_name]
            self.missions_collection = self.db['missions']
            self.detections_collection = self.db['detections']
            
            # Create indexes for better query performance
            self._create_indexes()
            
            print(f"✓ Connected to MongoDB: {self.database_name}")
            
        except (ConnectionFailure, ServerSelectionTimeoutError) as e:
            print(f"❌ Failed to connect to MongoDB: {e}")
            raise
    
    def _create_indexes(self):
        """Create indexes for optimized queries"""
        # Index on mission_id for fast lookups
        self.missions_collection.create_index("mission_id", unique=True)
        
        # Compound index for detections
        self.detections_collection.create_index([
            ("mission_id", 1),
            ("timestamp", -1)
        ])
        
        self.detections_collection.create_index("rack_id")
        self.detections_collection.create_index("item_code")
    
    def start_mission(self, mission_id: str) -> bool:
        """
        Start a new mission
        
        Args:
            mission_id: Unique mission identifier
            
        Returns:
            True if successful, False otherwise
        """
        try:
            mission_doc = {
                "mission_id": mission_id,
                "start_time": datetime.utcnow(),
                "end_time": None,
                "status": "in_progress",
                "total_detections": 0,
                "metadata": {
                    "created_by": "warehouse_rover",
                    "version": "1.0"
                }
            }
            
            self.missions_collection.insert_one(mission_doc)
            print(f"✓ Mission started: {mission_id}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to start mission: {e}")
            return False
    
    def end_mission(self, mission_id: str) -> bool:
        """
        End an active mission
        
        Args:
            mission_id: Mission identifier to end
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Count total detections
            detection_count = self.detections_collection.count_documents({
                "mission_id": mission_id
            })
            
            # Update mission
            result = self.missions_collection.update_one(
                {"mission_id": mission_id},
                {
                    "$set": {
                        "end_time": datetime.utcnow(),
                        "status": "completed",
                        "total_detections": detection_count
                    }
                }
            )
            
            if result.modified_count > 0:
                print(f"✓ Mission ended: {mission_id} ({detection_count} detections)")
                return True
            else:
                print(f"⚠️  Mission not found: {mission_id}")
                return False
                
        except Exception as e:
            print(f"❌ Failed to end mission: {e}")
            return False
    
    def add_detection(
        self,
        mission_id: str,
        rack_id: str,
        shelf_id: str,
        item_code: str,
        confidence: float,
        qr_data: str = "",
        additional_data: Dict = None
    ) -> Optional[str]:
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
            Detection ID (str) if successful, None otherwise
        """
        try:
            detection_doc = {
                "mission_id": mission_id,
                "rack_id": rack_id,
                "shelf_id": shelf_id,
                "item_code": item_code,
                "qr_data": qr_data,
                "confidence": confidence,
                "timestamp": datetime.utcnow(),
                "metadata": additional_data or {}
            }
            
            result = self.detections_collection.insert_one(detection_doc)
            return str(result.inserted_id)
            
        except Exception as e:
            print(f"❌ Failed to add detection: {e}")
            return None
    
    def get_detections_by_mission(self, mission_id: str) -> List[Dict]:
        """
        Get all detections for a mission
        
        Args:
            mission_id: Mission identifier
            
        Returns:
            List of detection documents
        """
        try:
            detections = list(self.detections_collection.find(
                {"mission_id": mission_id}
            ).sort("timestamp", 1))
            
            # Convert ObjectId to string for JSON serialization
            for det in detections:
                det['_id'] = str(det['_id'])
                # Convert datetime to ISO format string
                if 'timestamp' in det:
                    det['timestamp'] = det['timestamp'].isoformat()
            
            return detections
            
        except Exception as e:
            print(f"❌ Failed to get detections: {e}")
            return []
    
    def get_detections_by_rack(self, rack_id: str) -> List[Dict]:
        """
        Get all detections for a specific rack
        
        Args:
            rack_id: Rack identifier
            
        Returns:
            List of detection documents
        """
        try:
            detections = list(self.detections_collection.find(
                {"rack_id": rack_id}
            ).sort("timestamp", -1))
            
            # Convert ObjectId to string
            for det in detections:
                det['_id'] = str(det['_id'])
                if 'timestamp' in det:
                    det['timestamp'] = det['timestamp'].isoformat()
            
            return detections
            
        except Exception as e:
            print(f"❌ Failed to get detections by rack: {e}")
            return []
    
    def get_mission_summary(self, mission_id: str) -> Optional[Dict]:
        """
        Get mission summary
        
        Args:
            mission_id: Mission identifier
            
        Returns:
            Mission document or None
        """
        try:
            mission = self.missions_collection.find_one({"mission_id": mission_id})
            
            if mission:
                mission['_id'] = str(mission['_id'])
                
                # Convert datetime to ISO format
                if 'start_time' in mission:
                    mission['start_time'] = mission['start_time'].isoformat()
                if 'end_time' in mission and mission['end_time']:
                    mission['end_time'] = mission['end_time'].isoformat()
            
            return mission
            
        except Exception as e:
            print(f"❌ Failed to get mission summary: {e}")
            return None
    
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
        try:
            query = {"mission_id": mission_id} if mission_id else {}
            
            # Aggregate statistics
            pipeline = [
                {"$match": query},
                {"$group": {
                    "_id": None,
                    "total_detections": {"$sum": 1},
                    "avg_confidence": {"$avg": "$confidence"},
                    "unique_racks": {"$addToSet": "$rack_id"},
                    "unique_items": {"$addToSet": "$item_code"}
                }}
            ]
            
            result = list(self.detections_collection.aggregate(pipeline))
            
            if result:
                stats = result[0]
                return {
                    "total_detections": stats.get("total_detections", 0),
                    "average_confidence": round(stats.get("avg_confidence", 0), 3),
                    "unique_racks": len(stats.get("unique_racks", [])),
                    "unique_items": len(stats.get("unique_items", []))
                }
            else:
                return {
                    "total_detections": 0,
                    "average_confidence": 0.0,
                    "unique_racks": 0,
                    "unique_items": 0
                }
                
        except Exception as e:
            print(f"❌ Failed to get statistics: {e}")
            return {}
    
    def close(self):
        """Close MongoDB connection"""
        if self.client:
            self.client.close()
            print("✓ MongoDB connection closed")


# Test function
if __name__ == "__main__":
    import sys
    
    print("=" * 60)
    print("  MongoDB Manager Test")
    print("=" * 60)
    print()
    
    # Test with local MongoDB
    try:
        db = MongoDBManager(use_local=True)
        
        # Test mission
        test_mission = "TEST_MISSION_001"
        
        print("\n1. Starting test mission...")
        db.start_mission(test_mission)
        
        print("\n2. Adding test detections...")
        db.add_detection(test_mission, "R01", "S1", "ITEM001", 0.95, "R01-S1-ITEM001")
        db.add_detection(test_mission, "R01", "S2", "ITEM002", 0.88, "R01-S2-ITEM002")
        db.add_detection(test_mission, "R02", "S1", "ITEM003", 0.92, "R02-S1-ITEM003")
        
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
        sys.exit(1)
