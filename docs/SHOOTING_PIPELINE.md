# The Complete Shooting Pipeline — Vision to Score

This document traces the entire path from "camera sees an AprilTag" to "ball enters the hub," explaining every step, every offset, every conversion, and where each piece of code lives.

---

## Pipeline Overview

```
┌─────────────┐    ┌──────────────┐    ┌─────────────────┐    ┌──────────────┐    ┌─────────────┐
│  PhotonVision│───>│  Vision.java │───>│TurretAutoAimCmd │───>│ ShootCommand │───>│   Score!    │
│  (coprocessor)│    │  (subsystem) │    │  (turret control)│    │(hood+shooter │    │             │
│              │    │              │    │                  │    │  +indexers)  │    │             │
│ Detects tag  │    │ Filters,     │    │ Rotates turret   │    │ Sets hood    │    │ Ball enters │
│ Reports tx,  │    │ computes     │    │ to center tag,   │    │ angle +      │    │ hub         │
│ tag ID,      │    │ distance +   │    │ reports distance │    │ shooter speed│    │             │
│ 3D pose      │    │ diagnostics  │    │ for interp table │    │ from table,  │    │             │
│              │    │              │    │                  │    │ fires indexers│    │             │
└─────────────┘    └──────────────┘    └─────────────────┘    └──────────────┘    └─────────────┘
```

---

## Stage 1: PhotonVision — Tag Detection

**Where**: Runs on a coprocessor (Orange Pi / Raspberry Pi), NOT on the roboRIO.

**What happens**:
1. Camera captures a frame at 15-30 FPS
2. PhotonVision's AprilTag pipeline detects any visible AprilTags
3. For each detected tag, it computes:
   - **Tag ID** — which AprilTag (e.g., 9, 10, 25, 26)
   - **Yaw (`tx`)** — degrees the tag center is left/right of camera center. Positive = tag is to the right.
   - **3D camera-to-tag transform** — a full (X, Y, Z) translation from the camera lens to the tag, in meters
   - **Pipeline latency** — how long processing took (typically 20-40ms)
4. Results are sent to the roboRIO via NetworkTables

**Key detail**: The 3D transform is in the **camera's coordinate frame**:
- X = forward (out of lens)
- Y = left
- Z = up

---

## Stage 2: Vision.java — Filtering, Distance, Diagnostics

**File**: `subsystems/Vision.java`

### 2a: Receiving results

Every robot loop cycle (50 Hz / 20ms), `Vision.periodic()` checks for new PhotonVision results from the turret camera:

```java
latestResultTurret = turretCamera.getLatestResult();
```

It also tracks whether this frame is **fresh** (new since last cycle) by comparing timestamps.

### 2b: Dynamic camera transform

The turret camera moves with the turret. Its position relative to the robot changes every time the turret rotates. Vision.java recalculates this each cycle:

```
Robot Center
    │
    ├── ROBOT_TO_TURRET_PIVOT (static: -5.5" back, 6.5" left, 18" up, 180° yaw)
    │       │
    │       ├── [Turret rotation] (dynamic: current turret angle from Turret subsystem)
    │       │       │
    │       │       └── TURRET_PIVOT_TO_CAM (static: 0" forward, 5.25" left, 8" up, -15° pitch)
    │       │               │
    │       │               └── Camera lens position (in robot frame)
```

This is used for pose estimation (currently disabled) but the turret tracking doesn't use pose estimation — it uses the raw camera `tx` and `distance` directly.

### 2c: Distance calculation

**This is where the camera-to-bumper offset is applied.**

```java
// Vision.java — getTurretCameraDistanceToTarget()
Translation3d camToTag = latestResultTurret.getBestTarget()
        .getBestCameraToTarget().getTranslation();

// Step 1: Strip the Z (height) component — only horizontal distance matters
double hDist = Math.sqrt(camToTag.getX() * camToTag.getX()
        + camToTag.getY() * camToTag.getY());

// Step 2: Add the camera-to-bumper offset
return hDist + VisionConstants.CAMERA_TO_BUMPER_OFFSET_METERS;
```

**Why strip Z?** The camera and tag are at different heights. The 3D distance includes the vertical component, which would make close-range distances read too long. We only care about the horizontal distance for shooting — the ball travels horizontally.

**Why add the bumper offset?** The interpolation tables were calibrated by measuring **bumper-to-hub** with a tape measure. But the camera is ~20" behind the bumper (mounted on the turret). The offset bridges this gap:

```
Camera ──── hDist ────── Tag
   │                      │
   │◄─ OFFSET (20") ─►│  │
   │                   │  │
[camera]           [bumper] [hub]
   │                   │  │
   └── correctedDist ─────►│
```

`correctedDist = hDist + 0.508m` (20 inches)

This corrected distance is what gets fed to the interpolation tables.

### 2d: Diagnostics

`updateTurretDiagnostics()` populates telemetry fields every cycle:
- `rawCameraDist3d` — full 3D norm (including height)
- `horizontalCameraDist` — after stripping Z
- `correctedBumperDist` — after adding offset (this is what the interp table sees)
- `cameraToTargetX/Y/Z` — individual components for debugging
- `visionLatencyMs` — pipeline processing time
- `timeSinceLastFrameMs` — total time since last fresh frame

---

## Stage 3: TurretAutoAimCommand — Aiming the Turret

**File**: `commands/TurretAutoAimCommand.java`

This is the core control loop. It runs as the turret's **default command** — always active.

### 3a: Reading vision data

Each cycle, the command checks if there's a fresh vision frame:

```java
if (vision.isTurretResultFresh() && vision.turretCameraHasTargets()) {
    var target = vision.getTurretCameraBestTarget();
    int tagId = target.get().getFiducialId();
    
    // Only accept DS-facing hub tags (or any tag in bench test mode)
    if (VisionConstants.BENCH_TEST_ANY_TAG || VisionConstants.isDSFacingHubTag(tagId)) {
        visionTxDeg = target.get().getYaw();      // degrees off-center
        lastCameraDistance = vision.getTurretCameraDistanceToTarget(); // meters (corrected)
        freshVisionThisCycle = true;
    }
}
```

**Tag filtering**: `isDSFacingHubTag()` checks against `RED_DS_FACING_TAG_IDS = {9, 10}` and `BLUE_DS_FACING_TAG_IDS = {25, 26}`. Only tags facing the driver station are accepted. This prevents the turret from locking onto far-side hub tags which would give bad aim readings.

### 3b: State machine

```
MANUAL ──(tag detected)──> TRACKING
TRACKING ──(no tag for 0.15s)──> MANUAL
```

- **MANUAL**: Operator controls turret with joystick. No vision correction.
- **TRACKING**: Vision controls turret position. Operator joystick is ignored.

### 3c: The persistent target — how aiming actually works

This is the most important concept. The turret does NOT read its motor position each cycle and add a correction to it. Instead, it maintains a **persistent target position** (`persistentTargetRot`) that accumulates corrections:

```
Cycle 1: persistentTargetRot = 0.05 (seeded from motor position on TRACKING entry)
         Camera says tx = -5° (tag is 5° to the left)
         Correction = -(−5 / 360) × 1.0 = +0.0139 rot
         persistentTargetRot = 0.05 + 0.0139 = 0.0639
         Command turret to 0.0639

Cycle 2: Motor is still moving toward 0.0639 (at maybe 0.055)
         Camera says tx = -3° (closer, but not there yet)
         Correction = -(−3 / 360) × 1.0 = +0.0083 rot
         persistentTargetRot = 0.0639 + 0.0083 = 0.0722
         Command turret to 0.0722

Cycle 3: Camera says tx = -0.5° (within 1° deadband)
         No correction applied (|tx| < DEADBAND_DEG)
         persistentTargetRot stays at 0.0722
         Turret holds at 0.0722
```

**Why persistent target instead of re-reading motor position?**

If we read the motor position each cycle:
```
Cycle 1: Motor at 0.05. tx = -5°. Target = 0.05 + 0.0139 = 0.0639.
Cycle 2: Motor at 0.055 (still moving). tx = -3°. Target = 0.055 + 0.0083 = 0.0633.
         ← Target DECREASED even though we want to keep going right!
```

The motor is still catching up, so re-reading its position makes the target drift backward. The persistent target avoids this — corrections always accumulate forward.

### 3d: The tx-to-rotation conversion

```java
double txOffsetRot = -(visionTxDeg / 360.0) * TX_TO_ROT_GAIN;
```

Breaking this down:
- `visionTxDeg / 360.0` converts degrees to rotations (e.g., 5° = 0.0139 rot)
- The negative sign: if the tag is to the **right** (positive tx), the turret needs to rotate to bring the tag to center. Due to the camera moving with the turret and motor convention (Clockwise_Positive), a negative sign makes the turret move in the correct direction.
- `TX_TO_ROT_GAIN = 1.0` is a multiplier. At 1.0, the correction is 1:1 with the angular error. Increase to converge faster, decrease if oscillating.

### 3e: Gyro feedforward (optional)

When the robot rotates, the turret needs to counter-rotate to stay aimed. Without gyro FF, the camera detects the drift 1-2 frames later (~40-80ms). With gyro FF, the Pigeon2 IMU detects rotation instantly:

```java
double gyroRateDegPerSec = drivetrain.getPigeon2()
        .getAngularVelocityZWorld().getValueAsDouble();
gyroFeedforwardRot = (gyroRateDegPerSec / 360.0) * LOOP_PERIOD_SEC;
persistentTargetRot += gyroFeedforwardRot;
```

This adds a small turret rotation each cycle proportional to how fast the robot is turning. Controlled by `GYRO_FF_ENABLED` (default: false until tested).

### 3f: Soft limit clamping

After all corrections:
```java
persistentTargetRot = MathUtil.clamp(persistentTargetRot,
        TURRET_REVERSE_LIMIT,   // -0.58 rot
        TURRET_FORWARD_LIMIT);  //  0.20 rot
```

The turret can't aim past its physical limits. If the hub is behind the robot in an unreachable zone, the turret aims as far as it can.

### 3g: Commanding the motor

```java
turret.moveTurret(Rotations.of(persistentTargetRot));
```

This is called **every cycle** (even when there's no new camera frame). MotionMagic holds the turret at the target position, handling acceleration, deceleration, and position holding in firmware at 1 kHz. Between camera frames, the turret stays locked.

### 3h: Distance output

The corrected distance is stored and exposed via `getDistanceToHub()`:

```java
distanceToHubMeters = lastCameraDistance;
```

This value persists even after the tag is lost (stale but usable for a shot that was already lined up).

---

## Stage 4: ShootCommand — Hood, Shooter, Indexers

**File**: `commands/ShootCommand.java`

Triggered when operator holds **right trigger**. It runs in parallel with TurretAutoAimCommand (different subsystem requirements — ShootCommand requires hood, shooter, indexer; TurretAutoAimCommand requires turret).

### 4a: Interpolation table lookup

```java
double dist = distanceSupplier.getAsDouble(); // from TurretAutoAimCommand.getDistanceToHub()
hood.setHoodAngle(Rotations.of(calculator.getHoodAngleRot(dist)));
shooter.setShooterSpeed(RotationsPerSecond.of(calculator.getFlywheelRPS(dist)));
```

The `TurretAimingCalculator` has two `InterpolatingDoubleTreeMap` tables:

```
Distance (m) → Hood angle (rotations)      Distance (m) → Shooter speed (RPS)
1.232m → 0.000 rot                          1.232m → 45 RPS
2.216m → 0.016 rot                          2.216m → 50 RPS
3.232m → 0.031 rot                          3.232m → 55 RPS
4.191m → 0.046 rot                          4.191m → 60 RPS
5.207m → 0.050 rot                          5.207m → 63 RPS
6.223m → 0.053 rot                          6.223m → 69 RPS
7.239m → 0.055 rot                          7.239m → 74 RPS
```

Between calibration points, WPILib **linearly interpolates**. Example:
- Distance = 2.724m (halfway between 2.216 and 3.232)
- Hood = 0.016 + (0.031 - 0.016) × (2.724 - 2.216)/(3.232 - 2.216) = 0.0235 rot
- Shooter = 50 + (55 - 50) × 0.5 = 52.5 RPS

Outside the table range, values **clamp** to the nearest endpoint.

### 4b: Hood control

```java
hood.setHoodAngle(Rotations.of(calculatedAngle));
```

Hood.java clamps the setpoint to [0.0, 0.08] and commands MotionMagicVoltage. The hood moves to the correct launch angle for this distance.

### 4c: Shooter control

```java
shooter.setShooterSpeed(RotationsPerSecond.of(calculatedRPS));
```

Shooter.java clamps to MAX_SHOOTER_RPS (400) and commands VelocityVoltage. The flywheel spins up to the speed needed for this distance.

**Voltage limits**: PeakForwardVoltage = 10V (limits spin-up torque), PeakReverseVoltage = 0V (never brakes — coasts on friction when stopping).

### 4d: Indexer gating — the aim check

Indexers only fire when the turret is aimed:

```java
if (isAimed.getAsBoolean()) {
    indexer.setIndexerSpeed(-50, HORIZONTAL);
    indexer.setIndexerSpeed(50, VERTICAL);
    indexer.setIndexerSpeed(50, UPWARD);
}
```

`isAimed()` in TurretAutoAimCommand returns true when:
- State is TRACKING (camera sees a hub tag)
- `|visionTxDeg| < DEADBAND_DEG` (turret is centered within 1°)

This prevents firing balls when the turret isn't aimed. The ball only enters the shooter when all three indexer stages are running.

### 4e: On release

When the operator releases the trigger, `ShootCommand.end()` stops the shooter and all indexers. The flywheel coasts down (no reverse voltage).

---

## Stage 5: The Ball's Journey

```
1. Indexers push ball upward through the 3-stage system
2. Ball enters the spinning flywheel
3. Flywheel accelerates ball to launch velocity
4. Hood angle determines launch arc (steeper = more loft)
5. Ball exits the robot toward the hub
6. If distance + angle + speed are correct → SCORE
```

---

## Complete Data Flow Diagram

```
                    PhotonVision
                    (coprocessor)
                         │
                    tag ID, tx, 3D pose
                         │
                         ▼
                    Vision.java
                    ┌────────────────────────────┐
                    │ 1. Check: is result fresh?  │
                    │ 2. Extract 3D cam-to-tag    │
                    │ 3. Horizontal dist =        │
                    │    sqrt(X² + Y²)            │
                    │ 4. Corrected dist =         │
                    │    hDist + BUMPER_OFFSET     │
                    │ 5. Report tx (yaw degrees)  │
                    └──────┬──────────┬───────────┘
                           │          │
                      tx (deg)    distance (m)
                           │          │
                           ▼          │
                TurretAutoAimCommand  │
                ┌─────────────────┐   │
                │ 1. Filter tag ID│   │
                │ 2. Convert tx   │   │
                │    to rotation  │   │
                │ 3. Add to       │   │
                │    persistent   │   │
                │    target       │   │
                │ 4. Clamp to     │   │
                │    soft limits  │   │
                │ 5. Command      │   │
                │    MotionMagic  │   │
                │ 6. Store dist   │──►│
                └────────┬────────┘   │
                         │            │
                    turret rotates    │
                    to center tag     │
                                      │
                         ┌────────────┘
                         │
                         ▼
                    ShootCommand
                ┌──────────────────┐
                │ 1. Read distance │
                │ 2. Look up hood  │
                │    angle from    │
                │    interp table  │
                │ 3. Look up       │
                │    shooter speed │
                │    from table    │
                │ 4. Command hood  │
                │ 5. Command       │
                │    shooter       │
                │ 6. If isAimed(): │
                │    run indexers  │
                └──────────────────┘
                         │
                    ball launches
                         │
                         ▼
                       SCORE
```

---

## Where Every Offset Lives

| Offset | Value | File:Line | Purpose |
|---|---|---|---|
| `CAMERA_TO_BUMPER_OFFSET_METERS` | 20" (0.508m) | `VisionConstants.java:174` | Converts camera distance to bumper distance (matches interp table calibration) |
| `TURRET_AIM_OFFSET` | 0.0 | `TurretConstants.java:37` | Global offset for all turret setpoints (compensates gear slip) |
| `TX_TO_ROT_GAIN` | 1.0 | `TurretAutoAimCommand.java:73` | Scales tx-to-rotation conversion (faster/slower convergence) |
| `DEADBAND_DEG` | 1.0° | `TurretAutoAimCommand.java:76` | Vision tx below this is considered "aimed" — no correction applied |
| `ROBOT_TO_TURRET_PIVOT` | (-5.5", 6.5", 18", 180°) | `VisionConstants.java:135` | Physical location of turret rotation axis on robot |
| `TURRET_PIVOT_TO_CAM` | (0", 5.25", 8", -15° pitch) | `VisionConstants.java:151` | Camera position relative to turret pivot |
| Turret soft limits | [-0.58, +0.20] rot | `TurretConstants.java:30-31` | Physical turret rotation range (prevents hitting intake) |
| Hood range | [0.0, 0.08] rot | `TurretConstants.java:42-43` | Physical hood tilt range |

---

## Timing

| Event | Frequency | Latency |
|---|---|---|
| Camera frame capture | 15-30 Hz | ~33-66ms between frames |
| PhotonVision processing | per frame | 20-40ms pipeline |
| Vision.java reads result | 50 Hz | <1ms (NetworkTables read) |
| TurretAutoAimCommand executes | 50 Hz | <1ms (math + motor command) |
| MotionMagic PID loop | 1 kHz | <1ms (runs on motor controller) |
| Total: tag appears → turret starts moving | — | ~50-100ms |
| Total: turret locks on (from 10° offset) | — | ~0.5-1.0s |

---

## What Can Go Wrong (and what telemetry tells you)

| Problem | Telemetry symptom | Root cause |
|---|---|---|
| Turret aims at wrong tag | `trackedTagId` shows non-DS tag | Tag ID filter wrong, or `BENCH_TEST_ANY_TAG` is true |
| Distance reads wrong | `correctedBumperDist` doesn't match tape measure | `CAMERA_TO_BUMPER_OFFSET_METERS` needs adjustment |
| Turret oscillates | `turretErrorRot` alternates +/- | PID too aggressive (kP too high) or TX_TO_ROT_GAIN too high |
| Turret aims but offset | `visionTxDeg` settles at non-zero constant | Need `TURRET_AIM_OFFSET` or physical camera alignment |
| Balls fall short | `shooterVelocityRPS` < `shooterSetpointRPS` at time of fire | Shooter hasn't reached speed — need speed gate on indexers |
| Balls go over | Distance reads lower than actual | `CAMERA_TO_BUMPER_OFFSET_METERS` too large |
| Balls go left/right | `visionTxDeg` not near 0 at fire | Turret aim offset needed, or indexers fire before aimed |
| Nothing fires | `isAimed()` returns false | `visionTxDeg` > `DEADBAND_DEG` — turret hasn't locked on |
| Camera never detects tag | `trackedTagId` stays -1 | Check camera connection, PhotonVision pipeline, camera name |
