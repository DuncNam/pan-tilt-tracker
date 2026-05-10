# Pan-Tilt Laser Tracking System

A real-time computer vision tracking system that detects a colored object via USB webcam and commands a two-axis pan-tilt servo mount to follow it, with a laser pointer mounted to the assembly.

## System Overview
- **Vision:** OpenCV C++ color detection and centroid tracking at ≥30fps
- **Control:** PID controller driving pan/tilt servo angles
- **Hardware:** Raspberry Pi 5 → Arduino Uno → MG90S servos
- **Streaming:** Live annotated feed accessible via browser over WiFi

## Repository Structure
```
pan-tilt-tracker/
├── docs/                  # Requirements, architecture, assembly guide
├── src/                   # C++ vision code and Arduino sketch
├── media/                 # Demo video and assembly photos
└── calibration/           # HSV tuning values
```

## Hardware
| Component | Part |
|---|---|
| Compute | Raspberry Pi 5 2GB |
| Microcontroller | Arduino Uno R3 |
| Camera | Logitech Brio 101 |
| Servos | 2x MG90S Metal Gear |
| Laser | KY-008 650nm 5mW |

## Software Dependencies
- OpenCV 4.x (C++ API)
- Raspberry Pi OS 64-bit
- Arduino IDE + Servo.h

## Build Status
🔧 In progress — software stack complete, hardware integration pending

## Documentation
- [System Requirements](docs/SysReq_PanTiltTracker_v2.docx)
- [Assembly Guide](docs/assembly.md) *(in progress)*

## Demo
*Video coming once hardware integration is complete*