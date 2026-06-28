# Calibration & Tuning

Tuned parameter values for the pan-tilt tracker, with the reasoning behind each.

> **Source of truth:** the live values are defined in `src/tracker.cpp`. This file
> records the tuned values and the rationale for them. If the two ever disagree,
> the code is correct.

---

## HSV Color Bounds — Green Ping Pong Ball

Color-detection bounds used to mask the target. Tuned by hand against a green
ping pong ball under indoor lighting at the 4–8 ft demo distance.

| Channel | Low | High |
|---|---|---|
| Hue (H)        | 45  | 85  |
| Saturation (S) | 50  | 255 |
| Value (V)      | 30  | 255 |

OpenCV HSV ranges: H is 0–179, S and V are 0–255.

- **Hue 45–85** brackets the green band — narrow enough to reject the background,
  wide enough to hold the ball as lighting shifts across the frame.
- **Saturation floor of 50** is the sensitive bound. Motion blur and dim lighting
  desaturate the blob; too high a floor drops a fast-moving ball out of the mask.
  50 was the lowest value that still rejected washed-out background without
  false-locking on it.
- **Value floor of 30** rejects dark regions while keeping the lit ball.
- The mask is cleaned with one erode + one dilate iteration to remove speckle
  without shrinking the blob below the detection threshold.

---

## Detection & Control Thresholds

| Parameter | Value | Role |
|---|---|---|
| Min blob area | 300 px | Smallest contour accepted as a valid target |
| MAX_JUMP | 500 px | Largest frame-to-frame centroid move accepted |
| DEADBAND | 10 px | Error band inside which no correction is applied |
| Boresight X / Y | 60 / 35 px | Fixed laser-to-camera offset applied to the setpoint |

**Min blob area — 300 px.** Started at 500. Debugging showed the target was in the 
range of ~400–800 px at demo distance, dipping *below* 500 on routine frames, so the 
500 floor was rejecting the actual target. Lowering to 300 keeps those marginal 
frames. The tradeoff is the risk of accepting smaller noise blobs as targets. However, 
the HSV mask and erode/dilate cleanup keep the noise floor well under 300 on a clean 
background.

**MAX_JUMP — 500 px.** Started at 300. This is an outlier-rejection gate that discards 
large changes in centroid location between frames. Fast target swings produce large but 
legitimate centroid moves that the 300 gate was rejecting. Raising to 500 tolerates 
fast motion at the cost of weaker protection against false detections. This risk is 
accepted because the demoruns a single green target against a clean background.

**DEADBAND — 10 px.** Inside ±10 px of the setpoint, no correction is sent. This
suppresses servo jitter from the integer-pixel quantization and frame-to-frame
noise of the centroid when the target is essentially centered. Wide enough to stop
hunting on a still target, narrow enough not to leave a visible aiming gap.

**Boresight X / Y — 60 / 35 px.** The laser and camera are mounted apart, so the
laser does not strike where the camera centers. The setpoint is biased by a fixed
pixel offset to compensate. Calibrated by eye at the demo distance. This is a fixed
offset, so it is only correct near that range — see the range-parallax note in the
README's Known Limits.

---

## Tuning Method

Gains and thresholds were changed one at a time, each validated against a specific
physical symptom before the next change, and each PID term was gate-tested at zero
(confirmed to be a no-op) before a nonzero value was introduced. Tuned PID gains
(`KP = 0.04`, `KI = 0.012`, `KD = 0.015`) and the velocity-form derivation are
documented in the main [README](../README.md#control-approach).