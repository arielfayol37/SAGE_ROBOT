# Robot Tour AI Backend

A simple FastAPI-based backend for voice AI to control robot tours via WebSocket communication.

## 🚀 Features

- **WebSocket Communication**: Real-time bidirectional communication with robots
- **Simple Robot Management**: Track robot states and positions
- **REST API**: Basic HTTP endpoints for monitoring
- **Voice AI Ready**: Designed to work with your voice AI system

## 🏗️ Architecture

```
┌─────────────────┐    WebSocket    ┌─────────────────┐
│   Frontend      │ ←──────────────→ │   FastAPI       │
│   (Robot Sim)   │                 │   Backend       │
└─────────────────┘                 └─────────────────┘
                                              │
                                              ▼
                                     ┌─────────────────┐
                                     │   Your Voice    │
                                     │   AI (llm.py)   │
                                     └─────────────────┘
```

## 📋 Prerequisites

- Python 3.8+
- pip (Python package manager)

## 🛠️ Installation

1. **Navigate to backend directory:**

   ```bash
   cd backend
   ```

2. **Create virtual environment (recommended):**

   ```bash
   python -m venv venv

   # On Windows:
   venv\Scripts\activate

   # On macOS/Linux:
   source venv/bin/activate
   ```

3. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

## 🚀 Running the Backend

### Option 1: Using the startup script

```bash
python start.py
```

### Option 2: Direct uvicorn command

```bash
uvicorn main:app --host 0.0.0.0 --port 8002 --reload
```

### Option 3: Using main.py directly

```bash
python main.py
```

## 🌐 API Endpoints

### WebSocket

- **`ws://localhost:8002/ws/robot`** - Robot communication endpoint

### REST API

- **`GET /`** - Root endpoint with status
- **`GET /health`** - Health check
- **`GET /robots`** - List all connected robots
- **`GET /robots/{robot_id}`** - Get specific robot status
- **`POST /robots/{robot_id}/route`** - Send route to robot
- **`POST /robots/{robot_id}/cancel`** - Cancel robot route

## 📚 API Documentation

Once the server is running, visit:

- **Swagger UI**: http://localhost:8002/docs
- **ReDoc**: http://localhost:8002/redoc

## 🤖 WebSocket Message Types

### From Robot to Backend

- `robot_status` - Current robot status and position
- `robot_arrival` - Notification of destination reached

### From Backend to Robot

- `route_update` - New destinations and routes
- `route_cancel` - Cancel specific or all destinations
- `acknowledgment` - Message received confirmation

## 🎤 Voice AI Integration

This backend is designed to work with your voice AI system (`llm.py`):

1. **Robot Status**: Your AI can query robot status via REST API
2. **Route Control**: Send route updates via REST API or WebSocket
3. **Real-time Updates**: Instant destination changes based on voice commands

### Example Voice Commands

- "Take me to the robotics lab"
- "Cancel the current route and go to Room 101"
- "Add the computer science department after this stop"

## 🧪 Testing

### Manual Testing

1. Start the backend
2. Open frontend in browser
3. Click "Connect WebSocket"
4. Use robot controls to test communication

### API Testing

```bash
# Test health endpoint
curl http://localhost:8000/health

# Test robots endpoint
curl http://localhost:8000/robots
```

### WebSocket Testing

Use tools like:

- [WebSocket King](https://websocketking.com/)
- [Postman](https://www.postman.com/) (WebSocket support)
- Browser DevTools

## 🔧 Configuration

### Environment Variables

Create a `.env` file in the backend directory:

```env
HOST=0.0.0.0
PORT=8000
DEBUG=true
```

## 🚀 Production Deployment

For production deployment:

1. Set `DEBUG=false`
2. Use production WSGI server
3. Configure reverse proxy (nginx/Apache)

## 📝 Contributing

1. Fork the repository
2. Create feature branch
3. Implement your voice AI logic
4. Test thoroughly
5. Submit pull request

---

**Happy touring! 🎤🤖✨**
