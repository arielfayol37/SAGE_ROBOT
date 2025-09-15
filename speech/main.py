from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import json
import logging
from datetime import datetime
import uvicorn

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="Robot Tour AI Backend",
    description="Simple WebSocket backend for voice AI to control robot tours",
    version="1.0.0"
)

# Add CORS middleware for frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict to your frontend domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Simple data models
class RouteUpdate(BaseModel):
    destinations: List[str]
    clear_existing: bool = False

class RouteCancel(BaseModel):
    destination: Optional[str] = None

# Global state for connected robots
connected_robots: Dict[str, WebSocket] = {}
robot_states: Dict[str, Dict[str, Any]] = {}

# Simple WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}

    async def connect(self, websocket: WebSocket, robot_id: str):
        await websocket.accept()
        self.active_connections[robot_id] = websocket
        connected_robots[robot_id] = websocket
        robot_states[robot_id] = {
            "status": "connected",
            "last_seen": datetime.now(),
            "position": {"x": 0, "y": 0, "z": 0},
            "mode": "ai",
            "destinations": []
        }
        logger.info(f"Robot {robot_id} connected")

    def disconnect(self, robot_id: str):
        if robot_id in self.active_connections:
            del self.active_connections[robot_id]
        if robot_id in connected_robots:
            del connected_robots[robot_id]
        if robot_id in robot_states:
            del robot_states[robot_id]
        logger.info(f"Robot {robot_id} disconnected")

    async def send_personal_message(self, message: dict, robot_id: str):
        if robot_id in self.active_connections:
            try:
                await self.active_connections[robot_id].send_text(json.dumps(message))
            except Exception as e:
                logger.error(f"Failed to send message to robot {robot_id}: {e}")
                self.disconnect(robot_id)

manager = ConnectionManager()

# WebSocket endpoint for robot communication
@app.websocket("/ws/robot")
async def websocket_endpoint(websocket: WebSocket):
    robot_id = f"robot_{len(connected_robots) + 1}"
    
    try:
        await manager.connect(websocket, robot_id)
        logger.info(f"WebSocket connection established for {robot_id}")
        
        # Send initial connection confirmation
        await manager.send_personal_message({
            "type": "connection_established",
            "data": {
                "robot_id": robot_id,
                "message": "Connected to Tour AI Backend",
                "timestamp": datetime.now().isoformat()
            }
        }, robot_id)
        
        # Main message handling loop
        while True:
            try:
                # Receive message from robot
                data = await websocket.receive_text()
                message = json.loads(data)
                
                # Handle different message types
                await handle_robot_message(robot_id, message)
                
            except WebSocketDisconnect:
                logger.info(f"Robot {robot_id} disconnected")
                break
            except Exception as e:
                logger.error(f"Error handling message from robot {robot_id}: {e}")
                
    except Exception as e:
        logger.error(f"WebSocket error for robot {robot_id}: {e}")
    finally:
        manager.disconnect(robot_id)

# Simple message handler
async def handle_robot_message(robot_id: str, message: dict):
    """Handle incoming messages from robots"""
    try:
        message_type = message.get("type")
        message_data = message.get("data", {})
        
        logger.info(f"Received {message_type} from robot {robot_id}")
        
        # Update robot state
        if robot_id in robot_states:
            robot_states[robot_id]["last_seen"] = datetime.now()
            
            if message_type == "robot_status":
                # Update robot status
                robot_states[robot_id].update({
                    "position": message_data.get("position", robot_states[robot_id]["position"]),
                    "mode": message_data.get("mode", "ai"),
                    "status": message_data.get("status", "unknown"),
                    "destinations": message_data.get("destinations", [])
                })
                
            elif message_type == "robot_arrival":
                # Handle arrival notification
                print(f"Robot {robot_id} arrived at {message_data.get('destination')}")
                destination = message_data.get("destination")
                position = message_data.get("position")
                robot_states[robot_id].update({
                    "status": "arrived",
                    "arrived_at": destination,
                    "position": position or robot_states[robot_id]["position"],
                })
                logger.info(f"Robot {robot_id} arrived at {destination} at position {position}")
                
            # Send acknowledgment
            await manager.send_personal_message({
                "type": "acknowledgment",
                "data": {
                    "message_type": message_type,
                    "timestamp": datetime.now().isoformat()
                }
            }, robot_id)
            
    except Exception as e:
        logger.error(f"Error processing message from robot {robot_id}: {e}")

# Simple REST API endpoints
@app.get("/")
async def root():
    return {
        "message": "Robot Tour AI Backend",
        "version": "1.0.0",
        "status": "running",
        "connected_robots": len(connected_robots)
    }

@app.get("/health")
async def health_check():
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "connected_robots": len(connected_robots)
    }

@app.get("/robots")
async def get_robots():
    """Get status of all connected robots"""
    return {
        "connected_robots": len(connected_robots),
        "robot_states": robot_states
    }

@app.get("/robots/{robot_id}")
async def get_robot_status(robot_id: str):
    """Get status of specific robot"""
    if robot_id not in robot_states:
        raise HTTPException(status_code=404, detail="Robot not found")
    return robot_states[robot_id]

@app.post("/robots/{robot_id}/route")
async def send_route_to_robot(robot_id: str, route: RouteUpdate):
    """Send route update to specific robot"""
    if robot_id not in connected_robots:
        raise HTTPException(status_code=404, detail="Robot not connected")
    
    try:
        await manager.send_personal_message({
            "type": "route_update",
            "data": route.dict()
        }, robot_id)
        
        return {"message": f"Route sent to robot {robot_id}", "route": route.dict()}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to send route: {str(e)}")

@app.post("/robots/{robot_id}/cancel")
async def cancel_robot_route(robot_id: str, cancel: RouteCancel):
    """Cancel route for specific robot"""
    if robot_id not in connected_robots:
        raise HTTPException(status_code=404, detail="Robot not connected")
    
    try:
        await manager.send_personal_message({
            "type": "route_cancel",
            "data": cancel.dict()
        }, robot_id)
        
        return {"message": f"Route cancellation sent to robot {robot_id}"}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to cancel route: {str(e)}")

# Startup and shutdown events
@app.on_event("startup")
async def startup_event():
    logger.info("Robot Tour AI Backend starting up...")

@app.on_event("shutdown")
async def shutdown_event():
    logger.info("Robot Tour AI Backend shutting down...")

if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8002,
        reload=True,
        log_level="info"
    )
