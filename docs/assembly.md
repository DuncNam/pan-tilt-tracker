# Assembly Guide
## Pan-Tilt Laser Tracking System

As-built wiring and assembly reference for the tracker.

### Parts List

| Component | Part |
|---|---|
| Compute | Raspberry Pi 5 (2 GB) |
| Power (Pi) | GaN 27W USB-C PSU |
| Microcontroller | Arduino Uno R3 |
| Camera | Logitech Brio 101 |
| Servos | 2× MG996R metal gear |
| Bracket | Aluminum pan-tilt MG996R kit |
| Laser | KY-008 650 nm 5 mW |
| SD Card | SanDisk 32 GB A1 |
| Servo power | 5V 5A DC adapter |
| Barrel jack | 2.1 mm breadboard adapter |
| Wires | Dupont jumper set |
| Breadboard | 400-point |
| USB cable | USB-A to USB-B (Pi → Arduino) |

### Wiring

- **Servo power comes from the dedicated 5V 5A supply, not the Arduino.** The
  Arduino cannot source enough current for two MG996R servos under load.
- Servo signal wires: pan → Arduino D9, tilt → Arduino D10.
- **Common ground across all domains** — servo PSU ground, Arduino ground, and
  Pi ground are tied together. Without a shared ground the servo signal is
  referenced to the wrong rail.
- KY-008 laser: VCC → Arduino 5V, GND → Arduino GND, signal → Arduino D8.
- Pi ↔ Arduino: USB-A to USB-B cable. This carries both the serial link and
  Arduino logic power.

### Mounting

- The camera and laser are co-mounted on the pan-tilt platform and move as a
  single rigid unit, so the laser's aim tracks with the camera's view.
- The laser sits offset from the camera; this fixed offset is corrected in
  software with a boresight bias (see `calibration/tuning.md`).

### Design Notes

- **MG996R selected over MG90S.** The camera + laser co-mount load (~200 g) needs
  more torque than the 9 g MG90S can provide without running near stall. The
  MG996R (≈9 kg·cm at 4.8V) carries the load with margin.
- Servo travel is constrained to 10–170° in firmware as a safety margin against
  driving into the mechanical stops.
- Sub-degree pointing is achieved by commanding servo pulse width
  (`writeMicroseconds()`, 500–2500 µs) rather than integer degrees.

### Bring-Up Sequence

1. Flash the Arduino sketch and confirm it parses pan/tilt commands over the
   Serial Monitor.
2. Power the servos from the dedicated supply and verify physical movement from
   serial commands before mounting.
3. Mount the servos to the bracket, then co-mount the camera and laser on the
   platform.
4. Run boresight calibration at the intended demo distance.
5. Build and run the tracker on the Pi; confirm 30 fps capture and tracking.
6. Tune PID gains (see `calibration/tuning.md` for the method and final values).