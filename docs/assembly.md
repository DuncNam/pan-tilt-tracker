# Assembly Guide
## Pan-Tilt Laser Tracking System

### Current Status
Software stack complete as of 2026-05-10. Hardware integration pending parts arrival.

### Completed
- OpenCV C++ vision pipeline — color detection, centroid tracking, error calculation
- Arduino servo control sketch — serial command parsing, PWM output, safety constraints
- RPi serial integration — serial port configuration, proportional control loop
- RPi OS configured, OpenCV installed, code compiled and verified
- Arduino verified via Serial Monitor — confirmed parsing pan/tilt commands correctly

### Pending
- MG996R servos and aluminum bracket arrival (ordered 2026-05-09)
- Physical servo wiring and power supply test
- Camera and laser co-mounting on bracket
- Boresight calibration
- PID gain tuning

### Parts List
| Component | Part | Status |
|---|---|---|
| Compute | Raspberry Pi 5 2GB | ✅ In hand |
| Power | Argon GaN 27W USB-C PSU | ✅ In hand |
| Microcontroller | Arduino Uno R3 | ✅ In hand |
| Camera | Logitech Brio 101 | ✅ In hand |
| Servos | 2x MG996R Metal Gear | 🚚 Ordered |
| Bracket | Aluminum pan-tilt MG996R kit | 🚚 Ordered |
| Laser | KY-008 650nm 5mW | ✅ In hand |
| SD Card | SanDisk 32GB A1 | ✅ In hand |
| Power Supply | 5V 5A DC adapter | 🚚 Ordered |
| Barrel Jack | 2.1mm breadboard adapter | 🚚 Ordered |
| Wires | ELEGOO Dupont jumper set | ✅ In hand |
| Breadboard | ELEGOO 400-point | ✅ In hand |
| USB Cable | UGREEN USB-A to USB-B | ✅ In hand |

### Wiring Notes
- MG996R servo power from dedicated 5V 5A supply — NOT from Arduino
- Servo signal wires: pan → Arduino D9, tilt → Arduino D10
- All grounds share common connection — servo PSU GND, Arduino GND, RPi GND
- Arduino powered via USB-B from RPi
- KY-008 laser: VCC → Arduino 5V, GND → Arduino GND, Signal → Arduino D8
- RPi to Arduino serial: USB-A to USB-B cable

### Design Notes
- Camera must be co-mounted on pan-tilt assembly with laser — not fixed
- Boresight calibration required after assembly to align laser with frame center
- MG996R selected over MG90S after torque analysis — camera load ~200g requires >0.8kg/cm, MG90S operates at 40% stall torque under this load
- Servo angles constrained to 10-170° for safety margin

### Post-Assembly Steps
1. Wire servos and verify physical movement over serial
2. Mount servos to bracket
3. Mount Brio 101 and KY-008 to bracket platform
4. Run boresight calibration
5. Tune GAIN constant in tracker.cpp — start at 0.05, adjust based on response
6. Add I and D terms to upgrade from P to full PID controller