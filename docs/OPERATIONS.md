# Operational Guide — Getting the Robot Fully Running

Step-by-step instructions for powering on, testing, calibrating, and competing.

---

## Table of Contents
1. [Pre-Power Checklist](#1-pre-power-checklist)
2. [Boot Sequence](#2-boot-sequence)
3. [Elastic Dashboard Setup](#3-elastic-dashboard-setup)
4. [Subsystem Bring-Up (one at a time)](#4-subsystem-bring-up)
5. [Turret Tracking Verification](#5-turret-tracking-verification)
6. [Interpolation Table Calibration](#6-interpolation-table-calibration)
7. [Full Shooting Test](#7-full-shooting-test)
8. [Competition Checklist](#8-competition-checklist)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. Pre-Power Checklist

Before turning the robot on:

- [ ] **Turret facing straight forward**. The turret zeros at boot — if it's not forward, all positions will be wrong.
- [ ] **Hood at bottom hard stop**. The hood zeros at boot — it must be fully down.
- [ ] **Intake stowed**. The intake CANcoder is absolute but the motor zeros relative to it.
- [ ] **All CAN cables seated**. Check the "mechanisms" CAN bus connector especially.
- [ ] **Battery voltage >12.5V**. Low battery causes CAN timeouts and erratic motor behavior.
- [ ] **PhotonVision coprocessor powered and connected** (USB or Ethernet).

---

## 2. Boot Sequence

1. **Power on robot**
2. **Connect Driver Station** — verify all CAN devices report in Phoenix Tuner / Elastic
3. **Wait for console messages**:
   ```
   [Turret] Config readback:
     FeedbackSource: RotorSensor
     SensorToMech: 5.08
     FwdSoftLimit: enabled=true threshold=0.2
     RevSoftLimit: enabled=true threshold=-0.58
   [STARTUP] Turret unified command active (MANUAL/TRACKING)
   ```
4. **Check FaultMonitor** in Elastic for any motor faults
5. **Verify PhotonVision** — navigate to `http://<coprocessor-IP>:5800` and confirm the turret camera ("mugilanr") sees AprilTags

### Re-zero turret (if bumped)
Press **operator right bumper** to zero turret at current position. Make sure the turret is facing straight forward first.

---

## 3. Elastic Dashboard Setup

Add these widgets to your Elastic layout for critical telemetry:

### Turret tracking panel
- `TurretAutoAimCommand/state` — "MANUAL" or "TRACKING"
- `TurretAutoAimCommand/visionTxDeg` — should approach 0 when locked
- `TurretAutoAimCommand/distanceToHubMeters` — compare with tape measure
- `TurretAutoAimCommand/turretErrorRot` — should be <0.005 when settled
- `TurretAutoAimCommand/trackedTagId` — which tag (9/10 or 25/26)

### Vision diagnostics panel
- `Vision/correctedBumperDist` — the distance fed to interp tables
- `Vision/horizontalCameraDist` — camera-only distance (before offset)
- `Vision/visionLatencyMs` — pipeline latency (should be <50ms)
- `Vision/timeSinceLastFrameMs` — total frame age (should be <100ms)
- `Vision/cameraToTargetX/Y/Z` — raw components

### Turret motor panel
- `Turret/turretPositionRot` — current position
- `Turret/turretSetpointRot` — commanded position
- `Turret/turretErrorRot` — position error
- `Turret/faultForwardSoftLimit` — should be false
- `Turret/faultReverseSoftLimit` — should be false

### Shooter panel
- `Shooter/shooterSetpointRPS` — commanded speed
- `Shooter/shooterVelocityRPS` — actual speed
- `Shooter/shooterVoltageVolts` — should be 0-10V range

---

## 4. Subsystem Bring-Up

Test each subsystem individually before combining. **Enable the robot in teleop** for these tests.

### 4a. Drivetrain
1. Push left stick — robot should drive field-centric
2. Press **driver left bumper** to seed field-centric if heading is wrong
3. Press **driver A** to brake (wheels X-pattern)

### 4b. Turret (manual)
1. Push **operator right stick X** — turret should rotate smoothly
2. Watch `Turret/turretPositionRot` in Elastic
3. Push to soft limits — turret should stop at -0.58 and +0.20
4. If `faultForwardSoftLimit` or `faultReverseSoftLimit` goes true, limits are working
5. Test **D-pad presets**: Up=center(0.0), Right=0.15, Left=-0.20, Down=-0.45

### 4c. Hood
1. Press **operator X** — hood to 0.005 (down)
2. Press **operator Y** — hood to 0.040 (mid)
3. Press **operator B** — hood to 0.070 (up)
4. Watch `Hood/hoodPositionRot` — should match setpoints within 0.005

### 4d. Shooter
1. Press **operator right trigger** with NO tag visible — manual shoot at 40 RPS
2. Watch `Shooter/shooterVelocityRPS` — should ramp to ~40 and hold
3. Watch `Shooter/shooterVoltageVolts` — should stay under 10V
4. Release trigger — shooter should coast down (no reverse braking)

### 4e. Intake
1. Press **operator A** — intake deploys, roller starts
2. Press **operator A** again — intake stows, roller stops
3. Watch `Intake/pivotPositionRot` — should go to 0.653 (out) and 0.3 (in)

### 4f. Indexers
1. Hold **operator left trigger** — vertical indexer only
2. Hold **operator right trigger** (with tag visible so indexers fire) — all three run
3. Click **operator right stick** — all three reverse at 50% (clear jams)

---

## 5. Turret Tracking Verification

### Setup
1. Place an AprilTag (ID 9, 10, 25, or 26) at eye level, ~2m from robot
2. Or: set `BENCH_TEST_ANY_TAG = true` in `VisionConstants.java` to use any tag
3. Aim turret camera at the tag

### Test procedure
1. State should switch to "TRACKING" when tag is visible
2. `visionTxDeg` should decrease toward 0 as turret locks on
3. Move the tag slowly left/right — turret should follow
4. `turretErrorRot` should stay small (<0.01)
5. Block the camera — state should switch to "MANUAL" after 0.15s
6. Unblock — state should switch back to "TRACKING" instantly

### What to check if tracking doesn't work
- `Vision/turretCamConnected` — is the camera seen?
- `Vision/turretHasTargets` — does PhotonVision see any tags?
- `TurretAutoAimCommand/trackedTagId` — is it filtering the tag? (check DS-facing IDs)
- `Vision/visionLatencyMs` — is latency acceptable? (>100ms = problem)

---

## 6. Interpolation Table Calibration

### When to recalibrate
- After changing `CAMERA_TO_BUMPER_OFFSET_METERS`
- After moving the camera
- After mechanical changes to the shooter/hood
- If shots consistently miss at specific distances

### Procedure
1. Set up with a tape measure from **robot front bumper to hub base**
2. Hold **driver B** to enter calibration mode (CalibrationShootCommand)
3. Turret holds center, hood and shooter controlled via NetworkTables
4. In Elastic/NetworkTables, set:
   - `CalibrationShoot/hoodAngle` (rotations, 0.0 to 0.08)
   - `CalibrationShoot/shooterSpeed` (RPS, 40 to 80)
5. Feed balls and observe trajectory
6. Adjust until balls consistently score
7. Record: (distance_meters, hood_rot, shooter_RPS)
8. Repeat at distances: 1.2m, 2.2m, 3.2m, 4.2m, 5.2m, 6.2m, 7.2m
9. Update the tables in `TurretAimingCalculator.java`:
   ```java
   flywheelRPSTable.put(DISTANCE_METERS, SHOOTER_RPS);
   hoodAngleTable.put(DISTANCE_METERS, HOOD_ROT);
   ```

### Verifying distance accuracy
1. Place robot at known distance (tape measure)
2. Check `Vision/correctedBumperDist` in Elastic
3. Should match tape measure within ~10cm
4. If consistently off by a fixed amount, adjust `CAMERA_TO_BUMPER_OFFSET_METERS`

---

## 7. Full Shooting Test

### Vision-based shooting
1. Aim turret at hub tag — state = "TRACKING"
2. Wait for `turretErrorRot` < 0.005
3. Hold **operator right trigger**
4. Hood and shooter set from interp tables automatically
5. Indexers fire when turret is aimed (within 1° deadband)
6. Watch shots — adjust interp tables if needed

### Manual shooting
1. No tag visible — state = "MANUAL"
2. Use D-pad to aim turret manually
3. Use X/Y/B for hood angle
4. Hold **operator right trigger** — shoots at fixed 40 RPS, indexers always fire

---

## 8. Competition Checklist

### Before each match
- [ ] Turret facing forward, hood at bottom stop
- [ ] Battery >12.8V
- [ ] PhotonVision coprocessor on and streaming
- [ ] `BENCH_TEST_ANY_TAG` is **false** (only track DS-facing tags)
- [ ] `GYRO_FF_ENABLED` set as desired (false if untested)
- [ ] Verify correct alliance in Driver Station
- [ ] FaultMonitor shows no faults
- [ ] Auto routine selected in chooser

### Between matches
- Check for loose CAN connectors
- Check turret for gear slip (re-zero if needed: operator right bumper)
- Check hood for drift (re-power if zeroing is off)

---

## 9. Troubleshooting

### Turret won't track
| Symptom | Cause | Fix |
|---|---|---|
| State stays MANUAL | Camera can't see tag | Check PhotonVision web UI, verify camera connected |
| trackedTagId = -1 | Tag not in DS-facing list | Verify tag IDs or enable BENCH_TEST_ANY_TAG |
| Turret oscillates | kP too high or persistent target bug | Check turretErrorRot telemetry, reduce kP via SuperstructureTuner |
| Turret hits soft limit | Tag is behind robot | Turret range is 280° — some angles unreachable |

### Shooter issues
| Symptom | Cause | Fix |
|---|---|---|
| Slow spin-up | kV too low | Increase kV in SuperstructureTuner |
| Won't reach speed | Voltage capped at 10V | Speed may need lower target or higher kV |
| Reverse brakes | PeakReverseVoltage > 0 | Should be 0.0 — check TurretConstants |

### Distance reading wrong
| Symptom | Cause | Fix |
|---|---|---|
| Distance too short | CAMERA_TO_BUMPER_OFFSET too small | Measure camera-to-bumper and update constant |
| Distance too long | Offset too large | Reduce CAMERA_TO_BUMPER_OFFSET_METERS |
| Distance jumps | Multiple tags visible | Verify DS-facing tag filter is working |

### General CAN issues
- **Motor not responding**: Check CAN ID, verify in Phoenix Tuner
- **Erratic behavior**: Check battery voltage, CAN bus termination
- **Config not applied**: Check console for "Could not apply ... configs" errors
