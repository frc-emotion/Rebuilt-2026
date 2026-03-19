# Team Summary — What Changed & What Needs to Happen

Copy-paste this to your team. It covers everything we've done to the codebase and the exact steps needed to get the robot shooting at the hub from anywhere on the field.

---

## Part 1: Code Changes Made (already deployed)

### Dead code removed
- Deleted `moveHood.java`, `rotateTurret.java`, `runFlyWheel.java`, `TurretVisionTrackingTest.java` — unused commands that were never called
- Removed unused turret velocity PID gains (Slot 1) and unused `moveTurretWithWrap()`/`wrapTurretSetpoint()` methods
- Uncommented `SuperstructureTuner.checkForChanges()` in Robot.java so we can live-tune PID from Elastic

### Vision & distance fix
- **Distance calculation rewritten**: Now uses horizontal distance (strips height component) + a camera-to-bumper offset. Old method used raw 3D distance minus a hardcoded 3" which was inaccurate.
- **Camera-to-bumper offset added**: `CAMERA_TO_BUMPER_OFFSET_METERS = 20"` (placeholder — MUST be measured on robot). This converts camera distance to match how the interp tables were calibrated (bumper-to-hub with tape measure).
- **Diagnostic telemetry added** to Vision.java: `rawCameraDist3d`, `horizontalCameraDist`, `correctedBumperDist`, `cameraToTargetX/Y/Z`, `visionLatencyMs`, `timeSinceLastFrameMs`. All visible in Elastic.

### Tag filtering
- **DS-facing tag filter**: Turret only tracks tags that face the driver station (`RED_DS_FACING_TAG_IDS = {9, 10}`, `BLUE_DS_FACING_TAG_IDS = {25, 26}`). Prevents locking onto far-side hub tags that give bad aim readings. These IDs are placeholders — MUST be verified on the physical field.
- `BENCH_TEST_ANY_TAG = false` by default. Set to `true` for bench testing with any AprilTag.

### Turret tracking rewrite
- **Persistent target position**: The turret target accumulates corrections from vision instead of re-reading the motor position each cycle. This prevents oscillation caused by the motor still moving toward the previous target.
- **Turret error telemetry**: `turretErrorRot` logged for diagnosing tracking quality.
- **TX_TO_ROT_GAIN = 1.0**: Controls how aggressively the turret corrects based on camera yaw error. May need tuning.

### Gyro feedforward (OFF by default)
- When enabled (`GYRO_FF_ENABLED = true`), reads the Pigeon2's yaw rate and pre-rotates the turret to compensate for robot turning. Reduces tracking lag while driving. **Leave OFF until stationary tracking is verified.**

### Voltage limits on ALL motors
- Every motor now has `PeakForwardVoltage = 10.0V` and `PeakReverseVoltage = -10.0V` set in firmware config
- **Shooter exception**: `PeakReverseVoltage = 0.0V` — the flywheel never brakes, just coasts down on friction. Prevents fighting the flywheel's inertia.

---

## Part 2: What Needs to Happen on the Robot (in order)

**Estimated total time: 3-3.5 hours**

### Step 0: Hardware check (10 min)
- [ ] Power on with turret forward, hood at bottom stop
- [ ] Verify all CAN devices in Phoenix Tuner (IDs 20, 21, 22, 31, 32, 33, 50, 51, 52, 53, 54 on "mechanisms" bus)
- [ ] Check console for `[Turret] Config readback` — soft limits enabled, SensorToMech = 5.08
- [ ] Verify turret camera streaming in PhotonVision UI (`http://<coprocessor>:5800`)

### Step 1: Test turret position control (20 min)
- [ ] Test D-pad presets: Up=0.0, Right=+0.15, Left=-0.20, Down=-0.45
- [ ] Verify turret reaches setpoints within 1 second, error < 0.005 rot
- [ ] Verify soft limits stop turret at -0.58 and +0.20
- [ ] **Only change PID if something is broken** — existing values (kP=20, kI=5) should work

### Step 2: Test hood + shooter (15 min)
- [ ] Test hood presets: X=0.005, Y=0.040, B=0.070
- [ ] Verify hood reaches setpoints, holds position after release
- [ ] Test shooter at 40 RPS (hold operator RT with no tag visible)
- [ ] Verify shooter coasts down on release (no reverse braking)

### Step 3: Camera basics (10 min)
- [ ] Point turret at an AprilTag, verify state switches to "TRACKING"
- [ ] Verify `visionLatencyMs` < 50ms
- [ ] Block camera — state should go to "MANUAL" after ~0.15s
- [ ] If using non-hub tags for testing, set `BENCH_TEST_ANY_TAG = true`

### Step 4: Verify DS-facing tag IDs (15 min)
- [ ] On the physical field, identify which hub tags face each driver station
- [ ] Update `RED_DS_FACING_TAG_IDS` and `BLUE_DS_FACING_TAG_IDS` in VisionConstants.java
- [ ] Set `BENCH_TEST_ANY_TAG = false`
- [ ] Verify turret only tracks correct tags from scoring position

### Step 5: Measure camera-to-bumper offset (20 min)
- [ ] **Physically measure** from camera lens to front bumper along turret forward axis (in inches)
- [ ] Update `CAMERA_TO_BUMPER_OFFSET_METERS = Units.inchesToMeters(<YOUR_INCHES>)` in VisionConstants.java
- [ ] Place robot at 2m from a tag (tape measure from bumper), check `Vision/correctedBumperDist` ≈ 2.0m
- [ ] Repeat at 3m and 5m — error should be consistent (< ±0.1m)

### Step 6: Tune turret tracking (15 min)
- [ ] With tag at ~2m, verify turret locks on and `visionTxDeg` converges to < 1°
- [ ] If turret moves WRONG direction → flip sign in TurretAutoAimCommand line 234
- [ ] If convergence is slow → increase `TX_TO_ROT_GAIN` from 1.0 to 1.5
- [ ] If oscillating → decrease `TX_TO_ROT_GAIN` to 0.7 or increase `DEADBAND_DEG` to 2.0
- [ ] Test moving tag — turret should follow smoothly

### Step 7: Calibrate interpolation tables (45-60 min) — THE BIG ONE
- [ ] At each distance (1.5m, 2.0m, 3.5m, 4.0m, 5.0m, 6.0m, 7.0m):
  - Hold driver B (calibration mode)
  - Adjust hood angle + shooter speed in Elastic until balls score
  - Record (distance, hood_rot, shooter_RPS)
- [ ] Update tables in `TurretAimingCalculator.java`
- [ ] Deploy and verify 3/5 balls score at 3 different distances

### Step 8: Full integration shoot test (20 min)
- [ ] Place robot at 5 different positions/angles on the field
- [ ] At each: let turret auto-aim, hold RT, verify scoring
- [ ] Goal: 2/3 balls score at each position (10/15 minimum total)

### Step 9: Tracking while driving (15 min) — optional stretch
- [ ] Test tracking while driving slowly WITHOUT gyro FF (baseline)
- [ ] Set `GYRO_FF_ENABLED = true`, deploy
- [ ] Test tracking while driving — tx should be smaller than baseline
- [ ] If tx gets WORSE → flip sign in TurretAutoAimCommand line 225

---

## Part 3: Placeholder Values Cheat Sheet

These are the values you MUST measure/verify on the physical robot:

| What | Where | Current value | How to determine |
|---|---|---|---|
| Camera-to-bumper distance | `VisionConstants.java:174` | 20" | Tape measure: camera lens to front bumper |
| Red DS-facing tag IDs | `VisionConstants.java:230` | {9, 10} | Look at field — which hub tags face red DS |
| Blue DS-facing tag IDs | `VisionConstants.java:231` | {25, 26} | Look at field — which hub tags face blue DS |
| TX_TO_ROT_GAIN | `TurretAutoAimCommand.java:73` | 1.0 | Tune: increase if tracking is slow, decrease if oscillating |
| DEADBAND_DEG | `TurretAutoAimCommand.java:76` | 1.0 | Tune: increase if turret jitters at center |
| TURRET_AIM_OFFSET | `TurretConstants.java:37` | 0.0 | Set if turret consistently aims off-center |
| Interp table values | `TurretAimingCalculator.java:63-81` | From 2026-03-17 | Recalibrate with procedure in Step 7 |
| GYRO_FF_ENABLED | `TurretAutoAimCommand.java:87` | false | Set true after Step 8 passes |

---

## Part 4: How the Shooting System Works (short version)

```
Camera sees AprilTag → reports yaw error (tx) and 3D distance
    ↓
Vision.java strips height, adds bumper offset → corrected distance
    ↓
TurretAutoAimCommand adds -(tx/360) to persistent turret target
    ↓
Turret rotates via MotionMagic to center the tag (tx → 0)
    ↓
Operator holds right trigger → ShootCommand starts
    ↓
ShootCommand reads distance → looks up hood angle + shooter speed from interp table
    ↓
Hood moves to angle, shooter spins to speed
    ↓
When turret is aimed (tx < 1°) → indexers fire → ball launches → SCORE
```

---

## Part 5: Documentation

Full docs are in the `docs/` folder:

| File | What's in it |
|---|---|
| `SHOOTING_PIPELINE.md` | Complete technical walkthrough of the vision-to-score pipeline |
| `CONTROLS.md` | Every driver and operator button/stick with explanations |
| `SUBSYSTEMS.md` | Every subsystem — hardware, gear ratios, PID, associated commands |
| `PID_ANALYSIS.md` | Deep dive into every PID loop with tuning advice |
| `UNITS_AND_GEAR_RATIOS.md` | All units, gear ratios, CAN IDs |
| `OPERATIONS.md` | Boot sequence, competition checklist, troubleshooting |
| `CTRE_IMPROVEMENTS.md` | Future improvement opportunities |
| `tests/` | 10 ordered commissioning test procedures with telemetry instructions |
