#!/usr/bin/env python3
"""
Simple test script for Robot Tour AI Backend
"""

import asyncio
import websockets
import json
import requests
import time

# Configuration
BACKEND_URL = "http://localhost:8002"
WS_URL = "ws://localhost:8002/ws/robot"

async def test_websocket():
    """Test WebSocket connection and communication"""
    print("🔌 Testing WebSocket connection...")
    
    try:
        async with websockets.connect(WS_URL) as websocket:
            print("✅ WebSocket connected successfully!")
            
            # Wait for connection message
            try:
                message = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                data = json.loads(message)
                print(f"📨 Received: {data}")
            except asyncio.TimeoutError:
                print("⚠️  No initial message received")
            
            # Send robot status
            status_message = {
                "type": "robot_status",
                "data": {
                    "timestamp": "2024-01-01T00:00:00Z",
                    "position": {"x": 0, "y": 0, "z": 0},
                    "rotation": 0.0,
                    "mode": "ai",
                    "status": "idle",
                    "destinations": []
                }
            }
            
            await websocket.send(json.dumps(status_message))
            print("📤 Sent robot status")
            
            # Wait for acknowledgment
            try:
                response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                data = json.loads(response)
                print(f"✅ Received acknowledgment: {data}")
                
            except asyncio.TimeoutError:
                print("⚠️  No acknowledgment received within timeout")
            
    except Exception as e:
        print(f"❌ WebSocket test failed: {e}")

def test_rest_api():
    """Test REST API endpoints"""
    print("\n🌐 Testing REST API endpoints...")
    
    # Test root endpoint
    try:
        response = requests.get(f"{BACKEND_URL}/", timeout=5)
        if response.status_code == 200:
            print("✅ Root endpoint working")
            data = response.json()
            print(f"   Connected robots: {data.get('connected_robots', 0)}")
        else:
            print(f"❌ Root endpoint failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Root endpoint error: {e}")
    
    # Test health endpoint
    try:
        response = requests.get(f"{BACKEND_URL}/health", timeout=5)
        if response.status_code == 200:
            print("✅ Health endpoint working")
            data = response.json()
            print(f"   Status: {data.get('status', 'unknown')}")
        else:
            print(f"❌ Health endpoint failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Health endpoint error: {e}")
    
    # Test robots endpoint
    try:
        response = requests.get(f"{BACKEND_URL}/robots", timeout=5)
        if response.status_code == 200:
            print("✅ Robots endpoint working")
            data = response.json()
            print(f"   Connected robots: {data.get('connected_robots', 0)}")
        else:
            print(f"❌ Robots endpoint failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Robots endpoint error: {e}")

async def main():
    """Main test function"""
    print("🧪 Starting Robot Tour AI Backend Tests...")
    print(f"📍 Backend URL: {BACKEND_URL}")
    print(f"🌐 WebSocket URL: {WS_URL}")
    print("=" * 50)
    
    # Test REST API first
    test_rest_api()
    
    # Test WebSocket
    await test_websocket()
    
    print("\n" + "=" * 50)
    print("🏁 Tests completed!")
    print("\n💡 To see the full system in action:")
    print("1. Keep this test running")
    print("2. Open the frontend in your browser")
    print("3. Click 'Connect WebSocket'")
    print("4. Watch the real-time communication!")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n🛑 Tests interrupted by user")
    except Exception as e:
        print(f"\n❌ Test execution failed: {e}")
        print("\n💡 Make sure the backend is running:")
        print("   cd backend")
        print("   python start.py")
