# Measured Calibration Values

All values measured 2026-08-09 at 1280x720 MJPEG, 30 fps, exposure locked
to 10 ms manual.

## Camera focal length

**f_px = 1321 +/- 26 px** (0.0434 deg/px at frame centre)

Method: two marks 36 in apart on a flat wall, camera 71 in perpendicular,
measured separation 670 px. f_px = (670 x 71) / 36.

Implies 58.1 deg diagonal FOV, confirming the published Brio 101 spec of
58 deg.

Tool: `tools/capture_frame.cpp`

## Servo transfer function (pan)

**0.0971 deg/us** over 1250-1750 us

Ascending fit 0.09730, descending 0.09684, n=153 each, RMS residual 0.25 deg.
Linear across the measured range.

`tracker.cpp` assumed 0.090 deg/us (US_MIN/MAX 500-2500 over 180 deg).
Measured value is 7.9% higher, meaning a hidden 7.9% loop gain multiplier.

NOT VALIDATED outside 1250-1750 us. Extrapolating to the full rated range
assumes linearity that has not been checked at the extremes.

Tool: `tools/sweep_servo.cpp` -> `calibration/data/sweep_coarse.csv`

## Gear backlash (pan)

**0.99 deg = 11.5 us**

Mean hysteresis between ascending and descending sweeps across 51 matched
pulse widths.

This is the largest single term in the pointing error budget. At the 4 ft
operating range it is 21 mm, roughly half a ping-pong ball diameter. It is
incurred on every direction reversal, which for a pendulum target happens
twice per swing cycle.

Tool: `tools/sweep_servo.cpp` -> `calibration/data/sweep_coarse.csv`

## Servo internal deadband (pan)

**5 us ascending, 4 us descending = ~0.49 deg**

Longest run of single-microsecond commands producing no detectable motion.
Matches the MG996R datasheet nominal of 5 us. Detection floor of this method
is ~0.25 us equivalent.

Tool: `tools/sweep_servo.cpp` -> `calibration/data/sweep_fine.csv`

## Loop dead time

**Bracketed 37-71 ms**

Command issued to first observable centroid motion. Includes serial
transmission, Arduino processing, servo mechanical response, camera exposure,
and the full detection pipeline.

Quantised by the 33 ms frame period; this method cannot resolve finer.
Read from raw trajectories rather than the tool's summary statistics, which
suffer from baseline contamination.

Tool: `tools/measure_step.cpp`

## Servo slew rate (pan)

**85-105 deg/s**

Measured over 15 deg and 30 deg steps. The pendulum target moves estimated 
30-40 deg/s through the centre of its arc, so slew rate is not a limiting 
factor.

## Tilt axis geometry

Optical axis is horizontal at **tilt = 120 deg (1833 us)**, not the 90 deg
assumed in code. The tilt sign convention is inverted relative to pulse
width: increasing pulse width tilts DOWN.

From level, 833 us of upward travel and 167 us of downward travel remain
within the 1000-2000 us rated range. Pan calibration is only valid at the
elevation where it was taken, because pan and tilt are geometrically coupled.

## Error budget at 4 ft operating range

| Term | Angle | Linear |
|---|---|---|
| Command quantum (1 us) | 0.097 deg | 2.1 mm |
| Servo internal deadband | 0.49 deg | 10 mm |
| Software DEADBAND (15 px) | 0.65 deg | 14 mm |
| **Gear backlash** | **0.99 deg** | **21 mm** |

Sub-degree pointing repeatability is not achievable with MG996R servos.
A single direction reversal costs 0.99 deg. Backlash is compensable in
software with a measured feedforward of 11.5 us on detected reversal.
