# Robot Tour AI Backend

A real-time AI-controlled robot system using WebSockets for voice AI-guided tours of **Valparaiso University's College of Engineering**.

## Features

- **3D Robot Simulation**: Three.js-based frontend with realistic robot movement and pathfinding
- **Voice AI Integration**: Natural language tour guide using OpenAI GPT-4o
- **Real-time Communication**: WebSocket-based communication between frontend and AI backend
- **Intelligent Pathfinding**: A\* algorithm for optimal route planning
- **Destination Queue Management**: Sequential processing of tour destinations
- **Dynamic Status Updates**: Real-time robot position and status monitoring

## Voice AI Tour System

The system uses a voice-based AI (`llm.py`) that can:

- **Understand natural language requests** for tours and specific destinations
- **Provide detailed information** about engineering facilities (GEMC, Fites, SERF, iHub, Hesse)
- **Guide visitors** to other campus locations with basic information
- **Control robot movement** through tool calls to update/cancel routes
- **Dynamically inject robot status** into system prompts for context-aware responses

### Communication Flow

1. **User speaks** to the AI tour guide
2. **AI processes** the request and determines destinations
3. **AI sends route updates** to the FastAPI backend via tool calls
4. **Backend communicates** with the frontend robot simulation via WebSocket
5. **Robot executes** the tour route in the 3D environment
6. **Robot reports** arrival at destinations back to the backend
7. **AI receives** updated status for next interaction

### Message Types

- `route_update`: AI sends new destinations to the robot
- `route_cancel`: AI cancels specific or all destinations
- `robot_status`: Frontend reports current robot position and status
- `robot_arrival`: Frontend notifies when destination is reached
- `heartbeat`: Connection health monitoring

## Architecture

- **Frontend**: Three.js robot simulation (Port 8001)
- **Backend**: FastAPI WebSocket server (Port 8002)
- **AI**: Voice-based tour guide with OpenAI integration
- **Communication**: Bidirectional WebSocket for real-time updates

## Example Voice Commands

- "Take me on a tour of the engineering facilities"
- "Show me the solar furnace at SERF"
- "Let's visit the Innovation Hub makerspace"
- "Cancel the current route"
- "What engineering labs are available in the Fites building?"

## Future: Engineering School Environment

The system is designed to be easily adaptable for different engineering school environments by updating the facility descriptions and room layouts in the building generator and AI system prompt.

# Some Commands
Launch speech and teleop nodes

```
cd ~/Desktop/SAGE_ROBOT
./start_robot.sh
```

Launch lidar, robot_state_publisher, and nav2
```
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py

ros2 launch sllidar_ros2 sllidar_a1_launch.py frame_id:=lidar_link

ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/Desktop/SAGE_ROBOT/description/sage.urdf.xacro)"
ros2 launch slam_toolbox online_async_launch.py
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/home/agi/Desktop/SAGE_ROBOT/config/nav2_params.yaml \
  use_sim_time:=false
```

```
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```