# Robot Commissioning Tests — Ordered by Priority

This folder contains every test needed to get the robot's hub-targeting and shooting system fully operational. Tests are ordered by dependency — you **must** complete each test before moving to the next, because later tests depend on earlier ones being correct.

---

## Test Order

| # | File | What it validates | Estimated time |
|---|---|---|---|
| 0 | `00-HARDWARE-CHECK.md` | All motors, cameras, CAN devices online | 10 min |
| 1 | `01-TURRET-POSITION.md` | Turret PID tuning — setpoints, soft limits, response | 20 min |
| 2 | `02-HOOD-AND-SHOOTER.md` | Hood position + shooter velocity PID tuning | 15 min |
| 3 | `03-CAMERA-BASICS.md` | Turret camera sees AprilTags, latency is acceptable | 10 min |
| 4 | `04-TAG-ID-VERIFY.md` | Confirm DS-facing hub tag IDs on the physical field | 15 min |
| 5 | `05-DISTANCE-CALIBRATION.md` | Measure camera-to-bumper offset, validate distance | 20 min |
| 6 | `06-TURRET-TRACKING.md` | Turret locks onto tag, tracks smoothly, tx converges | 15 min |
| 7 | `07-INTERP-TABLE-CALIBRATION.md` | Calibrate hood + shooter at multiple distances | 45-60 min |
| 8 | `08-FULL-SHOOT-TEST.md` | End-to-end vision shoot at multiple positions | 20 min |
| 9 | `09-TRACKING-WHILE-DRIVING.md` | Gyro feedforward, turret maintains lock during motion | 15 min |

**Total estimated time: ~3-3.5 hours** for a complete commissioning from scratch.

---

## Placeholder Values That MUST Be Measured

These are all the values in the codebase that are marked as placeholders or need verification on the physical robot. Every test document tells you exactly which placeholders to resolve.

| Value | Current | File | Test # |
|---|---|---|---|
| `CAMERA_TO_BUMPER_OFFSET_METERS` | 20" (0.508m) | `VisionConstants.java:174` | 5 |
| `RED_DS_FACING_TAG_IDS` | {9, 10} | `VisionConstants.java:230` | 4 |
| `BLUE_DS_FACING_TAG_IDS` | {25, 26} | `VisionConstants.java:231` | 4 |
| `TURRET_ENCODER_OFFSET` | 0.0 | `TurretConstants.java:17` | 1 |
| `HOOD_ENCODER_OFFSET` | 0.0 | `TurretConstants.java:39` | 2 |
| `TURRET_AIM_OFFSET` | 0.0 | `TurretConstants.java:37` | 6 |
| `TX_TO_ROT_GAIN` | 1.0 | `TurretAutoAimCommand.java:73` | 6 |
| `DEADBAND_DEG` | 1.0 | `TurretAutoAimCommand.java:76` | 6 |
| `ROBOT_TO_TURRET_PIVOT` | measured but marked TODO | `VisionConstants.java:135` | 5 |
| `TURRET_PIVOT_TO_CAM` | measured but marked TODO | `VisionConstants.java:151` | 5 |
| MotionMagic cruise/accel/jerk | defaults | `TurretConstants.java:98-100` | 1 |
| `GYRO_FF_ENABLED` | false | `TurretAutoAimCommand.java:87` | 9 |
| Interp table data points | calibrated 2026-03-17 | `TurretAimingCalculator.java:63-81` | 7 |
| `INTAKE_ENCODER_OFFSET` | 0.0 | `IntakeConstants.java:91` | N/A |

---

## Telemetry Quick Reference

Every test uses these Elastic dashboard paths. Set up these widgets before starting:

### Turret tracking
- `TurretAutoAimCommand/state`
- `TurretAutoAimCommand/visionTxDeg`
- `TurretAutoAimCommand/distanceToHubMeters`
- `TurretAutoAimCommand/targetPositionRot`
- `TurretAutoAimCommand/currentPositionRot`
- `TurretAutoAimCommand/turretErrorRot`
- `TurretAutoAimCommand/trackedTagId`
- `TurretAutoAimCommand/gyroFeedforwardRot`

### Vision diagnostics
- `Vision/correctedBumperDist`
- `Vision/horizontalCameraDist`
- `Vision/rawCameraDist3d`
- `Vision/cameraToTargetX`
- `Vision/cameraToTargetY`
- `Vision/cameraToTargetZ`
- `Vision/visionLatencyMs`
- `Vision/timeSinceLastFrameMs`

### Turret motor
- `Turret/turretPositionRot`
- `Turret/turretSetpointRot`
- `Turret/turretErrorRot`
- `Turret/turretVelocityRPS`
- `Turret/turretVoltageVolts`
- `Turret/faultForwardSoftLimit`
- `Turret/faultReverseSoftLimit`

### Hood
- `Hood/hoodPositionRot`
- `Hood/hoodVelocityRPS`
- `Hood/hoodVoltageVolts`

### Shooter
- `Shooter/shooterSetpointRPS`
- `Shooter/shooterVelocityRPS`
- `Shooter/shooterVoltageVolts`
