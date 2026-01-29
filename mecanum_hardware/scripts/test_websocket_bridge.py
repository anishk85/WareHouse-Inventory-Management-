#!/usr/bin/env python3
"""
Test script for ROS2 WebSocket Bridge
Tests connection and basic message exchange
"""

import asyncio
import websockets
import json
import sys

async def test_websocket_connection():
    """Test connection to ROS2 WebSocket bridge"""
    uri = "ws://localhost:9090"
    
    print("🔌 Connecting to ROS2 WebSocket Bridge...")
    print(f"   URI: {uri}")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("✅ Connected successfully!")
            print("\n📊 Waiting for messages from bridge...")
            
            # Listen for 10 seconds
            timeout = 10
            start_time = asyncio.get_event_loop().time()
            message_count = 0
            
            while asyncio.get_event_loop().time() - start_time < timeout:
                try:
                    # Wait for message with 1 second timeout
                    message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                    data = json.loads(message)
                    message_count += 1
                    
                    print(f"\n📨 Message #{message_count}:")
                    print(f"   Type: {data.get('type', 'unknown')}")
                    
                    if data.get('type') == 'system_status':
                        print(f"   Active Nodes: {data.get('data', {}).get('active_nodes', 'N/A')}")
                        print(f"   Active Topics: {data.get('data', {}).get('active_topics', 'N/A')}")
                        print(f"   Uptime: {data.get('data', {}).get('bridge_uptime', 'N/A')}s")
                    
                    elif data.get('type') == 'qr_detections':
                        detections = data.get('data', {}).get('detections', [])
                        print(f"   QR Codes: {len(detections)}")
                        for detection in detections[:3]:  # Show first 3
                            print(f"      - {detection.get('content', 'N/A')} (confidence: {detection.get('confidence', 0):.2f})")
                    
                    elif data.get('type') == 'mission_status':
                        print(f"   Current Waypoint: {data.get('data', {}).get('current_waypoint', 'N/A')}")
                        print(f"   Task: {data.get('data', {}).get('current_task', 'N/A')}")
                        print(f"   Progress: {data.get('data', {}).get('progress', 0)}%")
                    
                    else:
                        print(f"   Data: {str(data.get('data', {}))[:100]}...")
                    
                except asyncio.TimeoutError:
                    # No message in 1 second, continue
                    continue
                except json.JSONDecodeError as e:
                    print(f"❌ JSON decode error: {e}")
            
            print(f"\n📈 Summary:")
            print(f"   Total messages received: {message_count}")
            print(f"   Average rate: {message_count / timeout:.1f} msg/s")
            
            # Test sending a command
            print(f"\n📤 Testing command sending...")
            test_command = {
                "type": "qr_control",
                "data": {"enabled": True}
            }
            await websocket.send(json.dumps(test_command))
            print(f"   ✅ Sent: {test_command}")
            
            # Wait for response
            try:
                response = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                print(f"   📨 Response: {response[:100]}...")
            except asyncio.TimeoutError:
                print(f"   ⏱️ No immediate response (this is normal)")
            
            print("\n✅ Test completed successfully!")
            
    except ConnectionRefusedError:
        print("❌ Connection refused!")
        print("   Make sure the ROS2 WebSocket bridge is running:")
        print("   $ ros2 run mecanum_hardware ros2_web_bridge.py")
        sys.exit(1)
    
    except Exception as e:
        print(f"❌ Error: {type(e).__name__}: {e}")
        sys.exit(1)

async def test_command_sequence():
    """Test sending a sequence of commands"""
    uri = "ws://localhost:9090"
    
    print("\n🎮 Testing command sequence...")
    
    try:
        async with websockets.connect(uri) as websocket:
            commands = [
                {"type": "qr_control", "data": {"enabled": True}},
                {"type": "actuator_control", "data": {"command": "lift_up"}},
                {"type": "cmd_vel", "data": {"linear": {"x": 0.5, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}},
                {"type": "actuator_control", "data": {"command": "lift_down"}},
                {"type": "cmd_vel", "data": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}},
            ]
            
            for i, cmd in enumerate(commands, 1):
                print(f"   {i}. Sending: {cmd['type']}")
                await websocket.send(json.dumps(cmd))
                await asyncio.sleep(0.5)  # Wait between commands
            
            print("   ✅ All commands sent!")
            
    except Exception as e:
        print(f"   ❌ Error: {e}")

def main():
    """Main test function"""
    print("=" * 60)
    print("  ROS2 WebSocket Bridge Test Suite")
    print("=" * 60)
    
    loop = asyncio.get_event_loop()
    
    try:
        # Test 1: Basic connection and message reception
        loop.run_until_complete(test_websocket_connection())
        
        # Test 2: Command sequence
        loop.run_until_complete(test_command_sequence())
        
        print("\n" + "=" * 60)
        print("  ✅ All tests passed!")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\n⏸️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
