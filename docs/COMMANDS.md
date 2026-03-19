# Command Reference — Rebuilt 2026

Every command in the robot, how it works, when it runs, and what subsystems it requires.

---

## Table of Contents
1. [TurretAutoAimCommand](#1-turretautoaimcommand)
2. [ShootCommand](#2-shootcommand)
3. [CalibrationShootCommand](#3-calibrationshootcommand)
4. [Intake Commands](#4-intake-commands)
5. [Indexer Commands](#5-indexer-commands)
6. [Manual Fallback Commands](#6-manual-fallback-commands)
7. [Climb Commands](#7-climb-commands)
8. [PathPlanner Named Commands](#8-pathplanner-named-commands)

---

## 1. TurretAutoAimCommand

**File**: `commands/TurretAutoAimCommand.java`
**Requires**: Turret
**Runs as**: Turret default command (always running)

### State machine
```
┌──────────┐   tag detected   ┌──────────┐
│  MANUAL  │ ───────────────> │ TRACKING │
│(joystick)│ <─────────────── │ (vision) │
└──────────┘  tag lost >0.15s └──────────┘
```

### MANUAL state
- Reads operator right-X joystick (deadband 0.08)
- Applies voltage directly to turret motor (±1.0 scaled)
- Keeps `persistentTargetRot` synced with actual turret position

### TRACKING state
Each 20ms cycle:
1. **Gyro feedforward** (when `GYRO_FF_ENABLED = true`): Reads Pigeon2 yaw rate, converts to turret rotation offset, pre-compensates for robot rotation
2. **Vision correction** (only on fresh camera frames): Reads turret camera `tx` (yaw error in degrees), converts to turret rotation offset via `-(tx / 360) * TX_TO_ROT_GAIN`, adds to `persistentTargetRot`
3. **Clamp** to soft limits
4. **Command** MotionMagic to `persistentTargetRot` every cycle (holds position between frames)

### Key design: Persistent target
The turret target position is NOT re-read from the motor each cycle. It accumulates corrections from vision. This prevents a feedback loop where:
- Motor is still moving toward target
- Code reads current position (not at target yet)
- Code adds correction on top of partially-reached position
- Target shifts backward, turret oscillates

### Tag filtering
Only accepts tags from `isDSFacingHubTag()` (red: 9,10 / blue: 25,26). Set `BENCH_TEST_ANY_TAG = true` for testing with any tag.

### Telemetry
| Field | Description |
|---|---|
| `state` | "MANUAL" or "TRACKING" |
| `visionTxDeg` | Camera yaw error (degrees) |
| `distanceToHubMeters` | Corrected distance for interp tables |
| `trackedTagId` | Which AprilTag we're locked onto |
| `targetPositionRot` | Commanded turret position |
| `currentPositionRot` | Actual turret position |
| `turretErrorRot` | target - actual |
| `gyroFeedforwardRot` | Per-cycle gyro compensation (DEBUG) |

---

## 2. ShootCommand

**File**: `commands/ShootCommand.java`
**Requires**: Indexer, Hood, Shooter
**Runs**: While operator right trigger is held

### Two modes

**Auto mode** (turret is TRACKING):
```java
new ShootCommand(indexer, hood, shooter, distanceSupplier, calculator, isAimedSupplier)
```
- Reads distance from turret camera
- Looks up hood angle and shooter speed from interpolation tables
- Sets hood angle and shooter speed every cycle
- Runs indexers only when `isAimed()` returns true (turret within 1° deadband)

**Manual mode** (turret is MANUAL):
```java
new ShootCommand(indexer, hood, shooter, 40.0)  // fixed 40 RPS
```
- Shooter spins at fixed 40 RPS
- Hood set to 0.0 (flat)
- Indexers always run (no aim gate)

### Indexer behavior during shooting
- Horizontal: -50 RPS (reversed due to mounting)
- Vertical: 50 RPS
- Upward: 50 RPS
- All three run simultaneously when firing

### On end
Stops shooter motor and all indexers. Flywheel coasts down (PeakReverseVoltage = 0V).

---

## 3. CalibrationShootCommand

**File**: `commands/CalibrationShootCommand.java`
**Requires**: Turret, Hood, Shooter, Indexer
**Runs**: While driver B is held

### Purpose
For tuning interpolation tables. Allows live adjustment of hood angle and shooter speed via NetworkTables/Elastic dashboard while the turret holds center position.

### Behavior
1. Turret moves to center (0.0)
2. Hood angle read from NetworkTables key
3. Shooter speed read from NetworkTables key
4. All indexers run to fire balls
5. Observe shot trajectory, adjust values in dashboard, repeat

---

## 4. Intake Commands

### IntakeOutCommand
**File**: `commands/intake/IntakeOutCommand.java`
**Requires**: Intake
**Trigger**: Operator A (toggle)

Deploys intake pivot to 0.653 rot and starts roller at 55 RPS. Finishes instantly — MotionMagic moves the pivot in the background. The roller runs until IntakeInCommand stops it.

### IntakeInCommand
**File**: `commands/intake/IntakeInCommand.java`
**Requires**: Intake
**Trigger**: Operator A (toggle)

Stows intake pivot to 0.3 rot and stops roller. Finishes instantly.

### IntakeIrrigateCommand
**File**: `commands/intake/IntakeIrrigateCommand.java`
**Requires**: Intake
**Trigger**: Currently disabled (commented out on left bumper)

Oscillates the intake pivot ±0.02 rot around stowed position using a triangle wave at 2.5 Hz. Purpose: dislodge balls stuck in the hopper. Runs while held, returns to stowed on release.

---

## 5. Indexer Commands

### runIndexer
**File**: `commands/indexer/runIndexer.java`
**Requires**: Indexer

Runs all three indexer motors at default speeds. Used in PathPlanner auto sequences. Runs until interrupted.

### stopIndexer
**File**: `commands/indexer/stopIndexer.java`
**Requires**: Indexer

Sets all three indexer speeds to 0. Finishes instantly (isFinished = true).

### Inline indexer controls (RobotContainer)
- **Left trigger**: Vertical indexer only (for manual ball staging)
- **Right stick click**: Reverse all indexers at 50% speed (clear jams)

---

## 6. Manual Fallback Commands

These are emergency-only commands for when auto-aim or closed-loop control fails.

### ManualTurretCommand
**File**: `commands/turret/ManualTurretCommand.java`
Direct voltage control of turret from joystick (±3V max, deadband 0.05). Not currently bound to any button — use via Elastic or code change if needed.

### ManualHoodCommand
**File**: `commands/turret/ManualHoodCommand.java`
Direct voltage control of hood from joystick (±3V max, deadband 0.05).

### ManualShooterCommand
**File**: `commands/turret/ManualShooterCommand.java`
Sets shooter speed proportional to joystick input (0 to MAX_SHOOTER_RPS).

---

## 7. Climb Commands

### LevelCommand
**File**: `commands/climb/LevelCommand.java`
Two-phase movement: first moves to `inner` position, waits for setpoint, then moves to `outer` position. Used as building block for climb sequences.

### Sequence
**File**: `commands/climb/Sequence.java`
Sequential command group that chains LevelCommands for multi-level climbs (1, 2, or 3 levels). All ClimbLevel positions are currently 0.0 (placeholder).

### ManualClimbCommand
**File**: `commands/climb/ManualClimbCommand.java`
Direct voltage control with gravity compensation (+0.5V kG offset). Runs while held.

**Note**: Climb is currently disabled (`climb = null` in RobotContainer).

---

## 8. PathPlanner Named Commands

Registered in `RobotContainer.registerNamedCommands()` for use in autonomous paths:

| Name | Description |
|---|---|
| `deployIntake` | IntakeOutCommand |
| `stowIntake` | IntakeInCommand |
| `runIndexers` | runIndexer (all three) |
| `stopIndexers` | stopIndexer (all three) |
| `startIntaking` | Deploy intake, then run indexers (sequential) |
| `stopIntaking` | Stow intake + stop indexers (parallel) |
| `visionShoot` | ShootCommand with interp tables (requires tracking) |
| `manualShoot` | ShootCommand at fixed 40 RPS |
| `spinUpShooter` | Pre-spin flywheel at 40 RPS |
| `feedWhenReady` | Run lower indexers immediately, upward only when shooter at speed |
| `revThenShoot` | Spin up + stage + feed when ready (single command) |
| `visionRevThenShoot` | Same but with interp-table hood/shooter + aim gate |
| `stopAll` | Stop shooter, indexers, stow intake |
| `enableVision` | No-op (auto-switches) |
| `disableVision` | No-op (auto-switches) |
