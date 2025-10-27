# SAGE — Autonomous Campus Tour Robot

**Senior Design Project (Valparaiso University)**  
**ROS 2 Humble · Jetson Orin Nano · STM32 · RPLIDAR A1M8 · WebRTC · Python**

SAGE is a Jetson-powered mobile robot designed to autonomously navigate indoor environments, stream live video to a browser, and respond naturally to voice commands.  
Built on **ROS 2 Humble**, the system combines **SLAM**, **navigation**, **real-time teleoperation**, and **voice interaction** into a single integrated stack.

---

## 🚀 Features

- **Autonomous Navigation & Mapping**  
  Uses SLAM Toolbox and Nav2 for indoor path planning and localization, powered by a precise URDF/TF/EKF pipeline.

- **WebRTC Teleoperation & Diagnostics**  
  Browser interface for operators and guides with live video, joystick/keyboard control, and system feedback.

- **Voice Interaction (LLM + Realtime STT/TTS)**  
  Enables conversational commands with ~2 s average latency using OpenAI’s LLM streaming APIs.

- **STM32 ↔ Jetson Bridge (Integrated in web_teleop_bridge)**  
  High-frequency serial link (~200 Hz) for differential-drive control, odometry feedback, and system state exchange.

- **Modular Launch & Automation**  
  `tmux`-based shell scripts simplify bring-up for different robot configurations (with/without IMU, SLAM, or voice).

---

## 🧠 System Architecture

**Jetson Orin Nano (ROS 2 Humble)**  
→ **Sensors**: RPLIDAR A1M8, wheel encoders, IMU  
→ **Perception / State**: SLAM Toolbox + EKF + TF/URDF  
→ **Planning / Control**: Nav2 navigation stack  
→ **Communication**: STM32 (serial) + WebRTC (teleop + video)  
→ **Interfaces**: Browser UI, Realtime STT/TTS + LLM  
→ **Operations**: `tmux` launch scripts · diagnostics · logging

---

## 📁 Repository Structure

SAGE_ROBOT/
├─ config/ # Nav2, EKF, and related ROS 2 parameters
├─ description/ # URDF/Xacro models and transforms
├─ maps/ # Saved maps (SLAM outputs)
├─ ros2_ws/
│ └─ src/
│ └─ web_teleop_bridge/ # WebRTC bridge + serial communication node (Jetson ↔ STM32)
├─ signaling/ # Signaling server for WebRTC
├─ speech/ # Realtime STT/TTS + LLM voice interface
├─ start_robot.sh # Bring-up script (speech + teleop nodes)
├─ slam_robot.sh # Bring-up with SLAM + Nav2
├─ start_robot_without_imu.sh # Alternate launch script
├─ slam_robot_without_imu.sh # Alternate SLAM setup without IMU
└─ README.md


---

## ⚙️ Quick Overview

- **Hardware:** Jetson Orin Nano, STM32 motor controller, RPLIDAR A1M8, encoders / IMU  
- **Software:** ROS 2 Humble · Python · WebRTC · OpenAI Realtime API  
- **Bring-Up:** Use provided shell scripts to start common configurations (SLAM / Navigation / Voice).  
  Full installation and setup guide will be added later.

---

## 🎙️ Voice & Teleoperation

- **Voice Interface:**  
  Realtime STT/TTS + OpenAI LLM streaming for conversational control; average latency ≈ 2 seconds.

- **WebRTC Teleop:**  
  Role-based browser interface with live video feed, keyboard/joystick input, and feedback from the STM32 bridge.  
  Includes signaling server (`signaling/`) and ROS 2 bridge node (`web_teleop_bridge/`).

---

## 🧩 Roadmap

- Add detailed **installation & build** instructions (Jetson setup, dependencies, `colcon` build, etc.)  
- Publish **maps**, **URDF diagrams**, and **Nav2 configurations**  
- Expand **voice command set** and optimize latency  
- Document **STM32 firmware** protocol and diagnostics  

---

## 📜 License

This project is released under the **MIT License**.  
See [`LICENSE`](LICENSE) for more information.

---

## 🙌 Acknowledgments

- Valparaiso University College of Engineering — Facilities and support  
- ROS 2, Nav2, and SLAM Toolbox open-source communities  
- Collaborators and testers involved in integration and demos  

---

*Developed as part of Valparaiso University’s Senior Design Program.*  
[Demo](https://www.youtube.com/watch?v=HRWUCX_4h8A&list=PLIcrH01m1D4cKdDTaD45W4qkC3aI_CjJB&index=1)] 
