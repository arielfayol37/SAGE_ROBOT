@echo off
echo 🤖 Starting Robot AI Backend...
echo.

REM Check if Python is available
python --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Python not found! Please install Python 3.8+ and try again.
    pause
    exit /b 1
)

REM Check if we're in the right directory
if not exist "main.py" (
    echo ❌ Error: main.py not found. Please run this script from the backend directory.
    pause
    exit /b 1
)

REM Check if virtual environment exists
if exist "venv\Scripts\activate.bat" (
    echo 🔄 Activating virtual environment...
    call venv\Scripts\activate.bat
) else (
    echo ⚠️  Virtual environment not found. Creating one...
    python -m venv venv
    call venv\Scripts\activate.bat
    echo 📦 Installing dependencies...
    pip install -r requirements.txt
)

echo 🚀 Starting backend server...
echo 📍 Server will run on http://localhost:8000
echo 🌐 WebSocket endpoint: ws://localhost:8000/ws/robot
echo 📊 API docs: http://localhost:8000/docs
echo.
echo 💡 Press Ctrl+C to stop the server
echo.

python start.py

pause
