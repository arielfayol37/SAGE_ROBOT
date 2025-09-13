#!/usr/bin/env python3
"""
Startup script for Robot Tour AI Backend
"""

import uvicorn
import sys
import os

def main():
    """Main startup function"""
    print("🤖 Starting Robot Tour AI Backend...")
    
    # Check if we're in the right directory
    if not os.path.exists("main.py"):
        print("❌ Error: main.py not found. Please run this script from the backend directory.")
        sys.exit(1)
    
    # Configuration
    host = os.getenv("HOST", "0.0.0.0")
    port = int(os.getenv("PORT", "8002"))
    reload = os.getenv("DEBUG", "true").lower() == "true"
    
    print(f"📍 Server will run on {host}:{port}")
    print(f"🔄 Auto-reload: {'enabled' if reload else 'disabled'}")
    print(f"🌐 WebSocket endpoint: ws://{host}:{port}/ws/robot")
    print(f"📊 API docs: http://{host}:{port}/docs")
    print("🚀 Starting server...")
    
    try:
        uvicorn.run(
            "main:app",
            host=host,
            port=port,
            reload=reload,
            log_level="info"
        )
    except KeyboardInterrupt:
        print("\n🛑 Server stopped by user")
    except Exception as e:
        print(f"❌ Error starting server: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
