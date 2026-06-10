# FUNCTIONALITY_INVENTORY.md — Rebuilt-2026 (Phase 1)

Complete behavioral inventory of the robot code as of commit `ede4cc5`. This is the contract for the refactor: every behavior here survives unless explicitly decided otherwise during design. Produced by parallel read-only ingestion; no robot code was modified.

## Phase 1 verification decisions (from team, 2026-06-09)

Checklist answers from the team. These are binding inputs to the design phase. NOT YET SIGNED OFF — awaiting the two-direction controls diff.

| # | Finding | Decision |
|---|---|---|
| 1 | Shoot-while-moving dead math (W11) | It never worked. **Fix it properly in the refactor, behind a toggle, default OFF.** |
| 2 | Lose-aim → 0 RPS re-opens indexer gate (W12) | **Confirmed bug.** Fix in refactor (feed must never run into a stopped/decaying flywheel). |
| 3 | indexerDefault missing addRequirements (W20) | Code was partially rewritten recently and likely hasn't run; several new commands were hand-written with such bugs. **Fix in refactor; treat untested commands with suspicion.** |
| 4 | Boot setup: turret straight forward, hood at bottom | **Confirmed.** Rotor-zero-at-boot is the real procedure. |
| 5 | MAX_POSE_AMBIGUITY = 1.0 (disabled) | **Re-enable at 0.3** in the refactor (not the old 0.15). |
| 6 | −0.2 m tag→hub lateral fudge (W24) | **Confirmed correct as tuned.** Sacred. |
| 7 | Operator modifiers (left stick click = turret manual, LB = passing) | **Confirmed.** |
| 8 | Dead/orphaned code | **Delete all**: LED (subsystem + constants), every mention/reference of Climb, runRoller, manualIndexer, the 5 orphan PathPlanner paths, unbound SysId routines. Goal: as clean as possible. |
| 9 | Blue 2 Depot_Outpost mid-field resetOdom; Blue 3 to Depot global 1.0 m/s | Team unsure — **treat both as probable bugs**; resolve in the auto phase. |
| 10 | CANivore "Persian  Canivore" double space | **Confirmed — matches the device.** Never "fix" it. |
| 11 | Vision never feeds odometry | **MAJOR NEW REQUIREMENT: implement complete vision pose estimation**, robust when limited to a single camera (single-tag handling, ambiguity gating at 0.3, std-dev scaling, `addVisionMeasurement` into the drivetrain). |
| 12 | Hood sag-follow default + ShootCommand leaves hood holding (W15/W32) | **Confirmed wanted.** But **AutoShootCommand is deprecated — delete it** (the `autoShoot`/`feedIndexers` auto pattern must be redesigned). |
| 13 | FaultMonitor referenced but absent | **Leftover — delete** the getters/comments that exist only for it. |
| 14 | TURRET_POS_RIGHT = 0.25 rot (90°) | **Value correct** (comment wrong). |
| 15 | reverseIndexers spins shooter forward at max during unjam (W19) | **Intended.** Preserve. |

---

**Robot summary:** Swerve (CTRE Tuner X generated, CANivore "Persian  Canivore") + turret/hood/shooter superstructure aimed by a turret-mounted PhotonVision camera, fed by a 3-stage indexer from a pivoting intake. Vision is a *targeting* sensor only — drivetrain pose is pure odometry. Telemetry is WPILib Epilogue (`@Logged`) + CTRE SignalLogger + a stock CTRE Telemetry class. LED and Climb are dead/orphaned.

---

## 1. Per-subsystem behavior tables

### 1.1 Drivetrain (CommandSwerveDrivetrain.java + Constants/TunerConstants.java)

Generated CTRE code + three hand additions. Per drivetrain policy, generated parts stay byte-identical; hand additions move to the adapter.

| Behavior | Trigger/condition | Constants involved | Location |
|---|---|---|---|
| Field-centric drive (default) | Default command, driver sticks | MaxSpeed = 5.85 m/s (`kSpeedAt12Volts`×1.0), MaxAngularRate = 0.75 rot/s = 4.712 rad/s, deadband 5% trans / 10% rot, OpenLoopVoltage, all 3 axes negated | RobotContainer.java:50-54, 118-121 |
| X-stance brake | Driver A whileTrue | — | RobotContainer.java:127 |
| Seed field-centric heading | Driver LB onTrue | — | RobotContainer.java:128 |
| Idle request while disabled | `RobotModeTriggers.disabled()` whileTrue, `ignoringDisable(true)` | — | RobotContainer.java:123-125 |
| Operator perspective (blue 0° / red 180°) | periodic, when not yet applied or disabled | kBlue/RedAlliancePerspectiveRotation | CommandSwerveDrivetrain.java:285-307 |
| Sim thread 4 ms | `Utils.isSimulation()` | kSimLoopPeriod = 0.004 | CommandSwerveDrivetrain.java:46, 309-322 |
| SysId routines (translation 4V/WPILOG, steer 7V, rotation π/6) | not bound to any button | — | CommandSwerveDrivetrain.java:69-124 |
| **HAND-ADDED** `getSwerveXSpeed()`/`getSwerveYSpeed()` | called by ShootCommand (dead math) | — | CommandSwerveDrivetrain.java:147-152 |
| **HAND-ADDED** `configurePathPlanner()` | called once from RobotContainer:107 | translation PID 10/0/0, rotation PID 7/0/0, `RobotConfig.fromGUISettings()`, alliance flip when Red, wheel-force FFs wired | CommandSwerveDrivetrain.java:257-283 |
| **HAND-ADDED** `@Logged` (Epilogue) | — | — | CommandSwerveDrivetrain.java:44 |
| `addVisionMeasurement` overrides (fpga→current time) | **ZERO CALLERS** — dead | — | CommandSwerveDrivetrain.java:334-364 |

TunerConstants (sacred, generated): drive CAN 26/25/23/18, steer 24/17/15/16, CANcoders 1/2/4/3 (FL/FR/BL/BR), Pigeon2 30; steerGains kP=100 kD=0.5 kS=0.1 kV=2.49; driveGains kP=0.1 kV=0.124; kDriveGearRatio=5.2734375, kSteerGearRatio=26.0909…, kCoupleRatio=3.375, wheel radius 2 in, modules ±10.91 in; offsets FL −0.205078125 / FR −0.364990234375 / BL 0.268798828125 / BR 0.4375; kInvertRightSide=true; kSlipCurrent=120 A; **hand-added** drive supply limit 60 A + stator 120 A in `driveInitialConfigs` (TunerConstants.java:77-84); kSpeedAt12Volts=5.85 m/s; bus `new CANBus("Persian  Canivore", "./logs/example.hoot")` — **double space is in the real name**.

### 1.2 Turret (Turret.java, TurretAutoAimCommand.java, util/TurretAimingCalculator.java)

| Behavior | Trigger/condition | Constants involved | Location |
|---|---|---|---|
| Boot: apply configs (5× retry, 0.1 s), zero rotor at current position, optimize bus, signal rates 50/50/4/4/4 Hz, soft-limit readback printout | constructor | TURRET_CONFIG (kP=40, kS=0.2, Brake, MM cruise 1.0/accel 2.0/jerk 20, RotorSensor, SensorToMechanism=122/24≈5.0833, soft limits +0.39/−0.73), TURRET_ENCODER_CONFIG | Turret.java:47-93, 159-173 |
| `moveTurret(Angle[, ff])`: wrap+clamp then MotionMagic; returns actually-commanded setpoint | TurretAutoAimCommand.execute, POV presets, calibration | TURRET_FORWARD_LIMIT=0.39, TURRET_REVERSE_LIMIT=−0.73 | Turret.java:97-122 |
| Manual open-loop: input × 3.0 V | operator left-stick-click held + right X | inline 3.0 | Turret.java:124-127 |
| `atTurretSetpoint()` | — | turretTolerance = 0.005 rot | Turret.java:129-132 |
| AUTO-AIM default command, 3 states: MANUAL (deadband 0.08), PASSING (latch on first fresh frame), TRACKING (visual servo: target = current + tx/360) | turret default command (only if turret && vision non-null) | DEADBAND_DEG=3.0, MANUAL_DEADBAND=0.08, GYRO_FF_ENABLED=true | TurretAutoAimCommand.java:78-126; RobotContainer.java:102-104 |
| Gyro feedforward: integrate Pigeon2 yaw deltas into setpoint each loop | every TRACKING/PASSING loop | — (sign/convention load-bearing) | TurretAutoAimCommand.java:173-183 |
| POV presets: up=0.0, down=−0.500, left=−0.250, right=0.25 rot | operator POV whileTrue | TURRET_POS_FORWARD/BACK/LEFT/RIGHT | RobotContainer.java:185-193 |
| Re-zero turret at current position | operator RB onTrue (no requirement!) | — | RobotContainer.java:171-174 |
| Interp tables: distance→flywheel RPS {1.55013…→41.5, 2.47698→43.5, 3.5→48.5, 4.620237→56.5}, distance→hood rot {same keys→0.000, 0.02, 0.03, 0.035}; validity window 1.0–7.0 m | `getFlywheelRPS`/`getHoodAngleRot` from Shoot/AutoShoot commands | calibrated 2026-03-17 from 48.5"/87.25"/127.25"/165" | TurretAimingCalculator.java:63-81, 139 |
| `isAimed()` = !manual && \|last tx\| < 3° | consumed by ShootCommand | DEADBAND_DEG=3.0 | TurretAutoAimCommand.java:204-206 |

### 1.3 Hood (Hood.java)

| Behavior | Trigger/condition | Constants involved | Location |
|---|---|---|---|
| Boot: configs (5× retry), **`setPosition(0)` — assumed at bottom hard stop**, signal rates 50/4/4/4 | constructor | HOOD_CONFIG (kP=100, **kI=50**, Brake, RotorSensor, SensorToMechanism=155/12≈12.9167, soft limits 0.0–0.08, MM 1.0/2.0/20), HOOD_ENCODER_CONFIG | Hood.java:31-49 |
| `setHoodAngle`: clamp [0.0, 0.08] then MotionMagic | Shoot/AutoShoot/Calibration commands, X/Y/B presets | HOOD_REVERSE/FORWARD_HARD_STOP = 0.0/0.08 | Hood.java:75-80 |
| Default command: continuously servo to *re-sampled* current position (sag-follow hold) | hood default | — | RobotContainer.java:91-94 |
| Presets: X=0.005 (down), Y=0.040 (mid), B=0.070 (up) | operator X/Y/B whileTrue | HOOD_POS_DOWN/MID/UP | RobotContainer.java:200-205 |
| `atHoodSetpoint()` | — | hoodTolerance = 0.005 rot | Hood.java:82-85 |

### 1.4 Shooter (Shooter.java)

| Behavior | Trigger/condition | Constants involved | Location |
|---|---|---|---|
| Boot: config (5× retry), signal rates 50/10/10 Hz | constructor | SHOOTER_CONFIG (kS=0.15 kV=0.12 kP=0.3, Coast, stator 160 A / supply 120 A, **PeakForward 12 V, PeakReverse 0 V** — "let friction stop it") | Shooter.java:27-39 |
| `setShooterSpeed`: clamp **upper bound only** (`Math.min`), VelocityVoltage | Shoot/AutoShoot/Calibration/reverseIndexers | MAX_SHOOTER_RPS = 400 | Shooter.java:59-63 |
| `atShooterSetpoint()` | gates indexer feed in ShootCommand | shooterTolerance = 1.67 RPS | Shooter.java:65-68 |

### 1.5 Intake (Intake.java + commands/intake/*)

| Behavior | Trigger/condition | Constants involved | Location |
|---|---|---|---|
| Pivot MotionMagic to angle; roller VelocityVoltage; roller stop = NeutralOut (coast) | commands | INTAKE_CONFIG (kP=19 kD=0.2, Brake, fused RemoteCANcoder ID 22, RotorToSensor 27.0, MM 1.5/2.5/40, soft limits 0.14–0.515), ROLLER_CONFIG (kS=0.15 kV=0.12 kP=0.3, Coast) | Intake.java:115-128 |
| `isOut()` = pivot > stow + 5° | computed in periodic; gates indexerDefault | INTAKE_IN_ANGLE=0.15, INTAKE_OUT_THRESHOLD_ROT=5/360 | Intake.java:62-63 |
| Over-travel recovery: re-command stow every 20 ms if trying-to-stow && pivot < 0.14 | periodic, subsystem-level | INTAKE_OVERTRAVEL_THRESHOLD=0.14 (== reverse soft limit) | Intake.java:67-77 |
| IntakeOutCommand: deploy to 0.51, start rollers (40 RPS) once within 15° (latched); on end: stop rollers + auto-restow to 0.15 | operator A **toggleOnTrue**; auto named `intakeOut`; path event markers | INTAKE_OUT_ANGLE=0.51, DEPLOY_TOLERANCE=15°, INTAKE_ROLLER_VELOCITY=40 | IntakeOutCommand.java:22-34; RobotContainer.java:141 |
| IntakeInCommand: stop roller, pivot to stow, finishes instantly | auto named `intakeIn` only (no teleop binding) | INTAKE_IN_ANGLE=0.15 | IntakeInCommand.java |
| runRoller: DEAD (binding commented out; no requirements, `intialize()` typo, no end()) | — | — | runRoller.java; RobotContainer.java:142 |

### 1.6 Indexer (Indexer.java + commands/indexer/*)

| Behavior | Trigger/condition | Constants involved | Location |
|---|---|---|---|
| 3 velocity-controlled stages: horizontal (31), vertical (32), upward (33) | commands | per-stage configs all kS=0.15 kV=0.12 kP=0.3, Coast, 60/30 A; horizontal inverted CW, others CCW | Indexer.java |
| indexerDefault: vertical at 35×0.75=26.25 RPS when `intake.isOut()`, else stop vertical. **No `addRequirements` call** | indexer default command | VERTICAL_INDEXER_SPEED=35 | indexerDefault.java:16-29; RobotContainer.java:96-98 |
| Manual vertical feed: vertical at 35 RPS | operator LT whileTrue (startEnd) | VERTICAL_INDEXER_SPEED=35 | RobotContainer.java:163-167 |
| reverseIndexers: all 3 stages at −50% (−17.5/−17.5/−50 RPS) **while shooter spins FORWARD at MAX_SHOOTER_RPS=400** | operator right-stick-click whileTrue; auto named `reverseIndexer` | HORIZONTAL/VERTICAL=35, UPWARD=100, MAX_SHOOTER_RPS=400 | reverseIndexers.java:27-32; RobotContainer.java:196 |
| manualIndexer: vertical at 35 — **never bound, dead** | — | — | manualIndexer.java |
| Auto `feedIndexers`: all 3 at full speed (35/35/100), ungated | auto named command | — | RobotContainer.java:246-253 |

### 1.7 Shooting commands

| Behavior | Trigger/condition | Constants involved | Location |
|---|---|---|---|
| ShootCommand (vision mode): if aimed && !passing → hood+shooter from interp tables at raw vision distance; if passing → hood 0.067 rot + 95 RPS hardcoded; else → shooter to `manualShooterRPS` (=0 in vision mode). Vertical indexer always on; horizontal+upward gated on `atShooterSetpoint()`. end(): stops shooter+indexer, **not hood** | operator RT whileTrue via `Commands.defer` (requirements only {indexer, hood, shooter}); auto named `shoot` (passing pinned `()->false`) | VERTICAL/HORIZONTAL/UPWARD_INDEXER_SPEED, FALLBACK_SHOOTER_RPS=50 (dead), passing 0.067/95 | ShootCommand.java:83-135; RobotContainer.java:148-156, 219-228 |
| Shoot-while-moving math: closing velocity → `effectiveDist` — **computed, never used** | every ShootCommand execute | shootingWhileMovingMultiplier=0.5 | ShootCommand.java:90-106 |
| AutoShootCommand: continuously track interp tables (no indexer requirement, so feed composes in parallel); `isReady()` = both at setpoint; end() stops shooter **and hood** | auto named `autoShoot` | — | AutoShootCommand.java |
| CalibrationShootCommand: NT-driven hood/RPS, turret pinned to 0, all indexers ungated, publishes live vision distance | driver B whileTrue — **commented out** | NT defaults hood 0.04 / 40 RPS | CalibrationShootCommand.java; RobotContainer.java:177-180 |

### 1.8 Vision (Vision.java + Constants/VisionConstants.java)

| Behavior | Trigger/condition | Constants involved | Location |
|---|---|---|---|
| Drain unread results, keep newest; sticky tag tracking (prefer currently-tracked tag); else lowest-ambiguity our-hub or our-passing tag | periodic | MAX_POSE_AMBIGUITY = 1.0 (**was 0.15 — effectively disabled**), TURRET_CAM_NAME="mugilanr" | Vision.java:60-110 |
| Hub output: camera→tag + TAG_TO_HUB_CENTER[tag] vector; distance = horizontal hypot; yaw negated (CW-positive) | hub tag classified | TAG_TO_HUB_CENTER: X=−0.604, lateral −0.2 or +0.156 (**−0.2 m fudge on every tag**) | Vision.java:135-145; VisionConstants.java:149-169 |
| Passing output: distance = hypot; **yaw from tag's surface NORMAL**, not bearing | passing tag classified | RED/BLUE_ZONE + NEUTRAL_*_SIDE tag ID sets; cross-alliance selection | Vision.java:146-157; VisionConstants.java:121-128 |
| Stale-distance hold: on target loss, hold last good distance (hub & passing separately); yaw implicitly freezes | no fresh frame / no targets | — | Vision.java:36-38, 76-80 |
| Tag classification precedence: hub checked BEFORE passing (tags 19/20/25/26 etc. overlap sets) | per frame | RED_HUB={2,3,4,5,8,9,10,11}, BLUE_HUB={18,19,20,21,24,25,26,27} | Vision.java:122-126; VisionConstants.java:64-71 |
| **No pose estimation. No `addVisionMeasurement` call anywhere. Drivetrain pose = odometry only.** | — | FRC2026_WELDED.json not loaded at runtime | grep-verified |

### 1.9 Dead / orphaned

| Item | State | Location |
|---|---|---|
| LED.java + LEDConstants.java | 100% commented out (intended: CANdle ID 0, hopper LEDs 8-48 intake-state strobe/alliance color, shooter LEDs 49-77 indexer-state yellow/green) | subsystems/LED.java, Constants/LEDConstants.java |
| ClimbConstants.java | Orphaned — no Climb subsystem exists; CLIMB_MOTOR=41 still in CANID | Constants/ClimbConstants.java |
| `manualIndexer`, `runRoller` | Dead commands (unbound / binding commented) | commands/ |
| 5 orphan PathPlanner paths | "Blue 1 to Neutral Pass 1" (+2 iterations), "Nuetral to Shoot", "New New Path" — abandoned neutral-pass routine | deploy/pathplanner/paths/ |
| SysId routines | Defined, never bound | CommandSwerveDrivetrain.java |
| `CANID.java` enum | Registry duplicating IDs declared per-constants-file; only referenced by RobotContainer import (unused) | Constants/CANID.java |

### 1.10 Robot lifecycle (Robot.java)

| Behavior | Trigger | Location |
|---|---|---|
| Epilogue root "Robot", importance DEBUG (MATCH_MODE=false), DataLogManager + SignalLogger started, Epilogue.bind | constructor | Robot.java:47-71 |
| `HootAutoReplay` (timestamp+joystick) updated before scheduler every loop | robotPeriodic | Robot.java:51-53, 74-77 |
| Auto scheduled from chooser; canceled on teleopInit | autonomousInit/teleopInit | Robot.java:92-113 |

---

## 2. Weird Stuff

Everything here is preserved unless the design phase explicitly decides otherwise. Quotes are verbatim from the current code.

### W1. Turret wrap-and-clamp (THE turret wrap) — Turret.java:111-122
```java
private double wrapAndClamp(double rot) {
    double raw = rot;
    if (rot < TurretConstants.TURRET_REVERSE_LIMIT) {
        rot += 1.0;
    } else if (rot > TurretConstants.TURRET_FORWARD_LIMIT) {
        rot -= 1.0;
    }
    rot = MathUtil.clamp(rot, TurretConstants.TURRET_REVERSE_LIMIT, TurretConstants.TURRET_FORWARD_LIMIT);
    turretWrapped = (rot != raw);
    wrappedSetpointRot = rot;
    return rot;
}
```
The turret has limited travel (−0.73 to +0.39 rot ≈ 403° total) because of its cable chain. When tracking pushes the setpoint past a limit, this adds/subtracts exactly one full rotation so the turret reaches the same physical heading from the other side of its wind-up range, then clamps as the final safety net (needed because total travel > 360° but < 720°, so the wrapped equivalent can still be out of range). Only a single ±1 correction is attempted. The wrap decision looks only at the setpoint, not the current position, so a wrap can command a near-full-rotation slew. `turretWrapped`/`wrappedSetpointRot` are CRITICAL-logged. This must survive exactly.

### W2. Wrapped setpoint fed back into the aim command — TurretAutoAimCommand.java:111, 125
```java
targetPositionRot = turret.moveTurret(Rotations.of(targetPositionRot)).in(Rotations);
```
`moveTurret` returns the post-wrap/clamp value and the command overwrites its own accumulator with it. Without this, the gyro-FF integration would wind the internal target unboundedly past the soft limits while the mechanism sits clamped. Load-bearing companion to W1.

### W3. Gyro feedforward by yaw-delta integration — TurretAutoAimCommand.java:173-183
```java
double currentYawDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();
double deltaDeg = currentYawDeg - lastGyroYawDeg;
lastGyroYawDeg = currentYawDeg;
gyroFeedforwardRot = deltaDeg / 360.0;
targetPositionRot += gyroFeedforwardRot;
```
Each loop the chassis yaw change is added to the turret setpoint so the turret stays field-pointed between vision frames. Integrating measured deltas is drift-free vs. the gyro and immune to loop-timing jitter. The **sign (addition, not subtraction) encodes the turret/gyro rotation convention and is load-bearing**. A commented-out alternate using `getAngularVelocityZWorld() * LOOP_PERIOD_SEC * 1.75` (a hand-tuned lead fudge) was tried and abandoned.

### W4. `applyOmega()` is a deliberate stub — TurretAutoAimCommand.java:132-136
```java
private void applyOmega() {
    return;
    // targetPositionRot += (omega * TurretConstants.omegaFeedforwardMultiplier);
}
```
Chassis omega is still sampled and CRITICAL-logged every TRACKING loop, but velocity-based lead compensation is disabled (`omegaFeedforwardMultiplier = 0.05` sits unused). Kept as scaffolding/telemetry for retuning.

### W5. Vision frame dedupe shares one timestamp between hub and passing reads — TurretAutoAimCommand.java:138-154
```java
double ts = vision.getTurretResultTimestamp();
if (ts == lastVisionTimestamp) return;
lastVisionTimestamp = ts;
if (!vision.isSeeingHubTag()) return;
```
Exact-timestamp dedupe stops a 50 Hz robot loop from applying the same ~25 Hz camera frame's tx twice (which would double-correct and oscillate). Quirk: the timestamp is consumed *before* the tag check, so a fresh frame without the wanted tag burns the timestamp; `readPassing()` is a clone sharing `lastVisionTimestamp`, so hub and passing frames dedupe against each other.

### W6. Pure visual servoing — tx fused onto current position — TurretAutoAimCommand.java:121-123
```java
if (freshVisionThisCycle) {
    targetPositionRot = currentPositionRot + visionTxDeg / 360.0;
}
```
On a fresh frame: target = current position + camera yaw error. The pose-based `TurretAimingCalculator.calculate()` path exists but is **never called** — aiming is vision-tx + gyro-delta only. No latency compensation (tx measured at capture is applied against current position).

### W7. Manual turret mode rebases the closed-loop target — TurretAutoAimCommand.java:98-101
```java
double input = MathUtil.applyDeadband(joystickSupplier.getAsDouble(), MANUAL_DEADBAND);
turret.setTurretVoltage(MathUtil.clamp(input, -1, 1));
targetPositionRot = turretPos;
```
During open-loop manual (±3 V), the target continuously tracks actual position so releasing manual holds in place — no snap-back. Soft limits still protect manual mode at the firmware level.

### W8. Turret zeroed at boot via rotor sensor; CANcoder is telemetry-only — Turret.java:54-58
```java
// Zero at current position (assumed straight-forward at boot, like hood).
turretMotor.setPosition(0);
```
Feedback is RotorSensor (ratio 5.0833) zeroed wherever the turret physically sits at power-on. The turret CANcoder (ID 53) is configured but only read for DEBUG telemetry at 4 Hz. **The robot must be set up with turret straight-forward before power-on** or every setpoint, soft limit, and wrap is offset. Same pattern for hood (Hood.java:38-39, "assumed to be bottom/home"; hood CANcoder ID 54 also unfused). Operator RB re-zeroes the turret mid-match (RobotContainer.java:171-174) — directly pokes `setPosition(0)` with no command requirement.

### W9. Boot config readback paranoia — Turret.java:159-173
Prints FeedbackSource, SensorToMechanism, and both soft-limit enables/thresholds to stdout after config-apply, verifying the soft limits actually stuck (real CTRE failure mode; a turret without soft limits destroys its cable chain).

### W10. `isAimed()` gates on last-seen vision tx, not turret error — TurretAutoAimCommand.java:204-206
```java
public boolean isAimed() {
    return !manualOverride.getAsBoolean() && Math.abs(visionTxDeg) < DEADBAND_DEG;
}
```
Shoot-readiness = last vision yaw error < 3°. `visionTxDeg` initializes to 0, so before the first frame this reads true — a false-ready window.

### W11. Shoot-while-moving compensation computed but never used — ShootCommand.java:90-106
```java
double vClosing = vx * Math.cos(turretAngleRad) + vy * Math.sin(turretAngleRad);
double effectiveDist = dist - vClosing * TurretConstants.shootingWhileMovingMultiplier;
...
hood.setHoodAngle(Rotations.of(calculator.getHoodAngleRot(dist)));        // <- dist, not effectiveDist
shooter.setShooterSpeed(RotationsPerSecond.of(calculator.getFlywheelRPS(dist)));
```
Closing-velocity math projects chassis speed onto the turret heading and shortens the lookup distance — but the lookups use raw `dist`. Either deliberately reverted or never finished. The drivetrain and turret reads in ShootCommand exist only to feed this dead math.

### W12. Not-aimed fallback commented out → 0 RPS can ungate the feed — ShootCommand.java:103-118
```java
// } else if (useInterpTables) {
//     shooter.setShooterSpeed(RotationsPerSecond.of(FALLBACK_SHOOTER_RPS));
} else {
    if(passing){ hood.setHoodAngle(Rotations.of(0.067)); shooter.setShooterSpeed(RotationsPerSecond.of(95)); }
    else { shooter.setShooterSpeed(RotationsPerSecond.of(manualShooterRPS)); }
}
```
In vision mode `manualShooterRPS` = 0, so losing aim commands 0 RPS; once the flywheel decays within 1.67 RPS of zero, `atShooterSetpoint()` goes TRUE and the horizontal/upward indexers feed into a dead flywheel. The javadoc still describes the old FALLBACK behavior. Looks like an unintended interaction from an in-season hack.

### W13. Spin-up gating with vertical stage exempt — ShootCommand.java:120-128
```java
indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL);
if (shooter.atShooterSetpoint()) {
    indexer.setIndexerSpeed(..., IndexerType.HORIZONTAL);
    indexer.setIndexerSpeed(..., IndexerType.UPWARD);
} else { stop both }
```
Vertical pre-stages balls unconditionally; the stages that push into the flywheel run only at speed. Instantaneous (no debounce), re-closes mid-volley if recoil drops velocity out of tolerance — intentional anti-dribble.

### W14. Hardcoded passing shot beats everything — ShootCommand.java:111-114
Passing (operator LB, vision mode only) bypasses the interp tables entirely: hood 0.067 rot, 95 RPS. Condition ordering makes passing defeat aimed. Auto pins passing to `() -> false`.

### W15. Hood end-behavior asymmetry — ShootCommand.java:131-135 vs AutoShootCommand.java:55-59
Teleop ShootCommand `end()` stops shooter+indexer but **leaves the hood holding** its last position; AutoShootCommand stops shooter **and hood**. Deliberate (javadoc: hood stays where operator left it).

### W16. Intake over-travel recovery lives in the subsystem, not a command — Intake.java:67-77
```java
boolean tryingToStow = Math.abs(currentSetpoint.in(Rotations) - IntakeConstants.INTAKE_IN_ANGLE.in(Rotations)) < 0.01;
if (tryingToStow && pivotPositionRot < IntakeConstants.INTAKE_OVERTRAVEL_THRESHOLD) {
    intakeMotor.setControl(intakeMotionRequest.withPosition(IntakeConstants.INTAKE_IN_ANGLE));
    overtravelRecovery = true;
}
```
If collisions/momentum shove the stowed intake past 0.14 rot toward the robot interior, periodic re-commands the stow position every 20 ms so PID fights back before mechanical jam. Only runs when stowing — never blocks deploy. Needed because IntakeInCommand finishes instantly, leaving no command to fight back.

### W17. Deploy-then-spin latch with deliberately loose tolerance — IntakeOutCommand.java:22-34
Rollers start only once the pivot is within DEPLOY_TOLERANCE (15° vs the 5° default tolerance — "rollers start sooner on deploy"), then latch on so pivot bounce can't stop them. `end()` doubles as auto-restow (toggleOnTrue: second press = interrupt = stop rollers + stow), making IntakeInCommand teleop-redundant.

### W18. `isOut()` means "barely off stow", not "deployed" — Intake.java:62-63
```java
intakeIsOut = pivotPositionRot > IntakeConstants.INTAKE_IN_ANGLE.in(Rotations) + IntakeConstants.INTAKE_OUT_THRESHOLD_ROT;
```
"Out" = >5° from stow. The indexer default keys off this, so the vertical indexer (at 75% = 26.25 RPS) runs during the entire pivot transit in both directions.

### W19. reverseIndexers spins the shooter FORWARD at max while reversing all stages — reverseIndexers.java:27-32
```java
m_indexerSubsystem.setIndexerSpeed(-IndexerConstants.HORIZONTAL_INDEXER_SPEED * 0.5, IndexerType.HORIZONTAL);
m_indexerSubsystem.setIndexerSpeed(-IndexerConstants.VERTICAL_INDEXER_SPEED * 0.5, IndexerType.VERTICAL);
m_indexerSubsystem.setIndexerSpeed(-IndexerConstants.UPWARD_INDEXER_SPEED * 0.5, IndexerType.UPWARD);
m_shooterSubsystem.setShooterSpeed(RotationsPerSecond.of(TurretConstants.MAX_SHOOTER_RPS));
```
Anti-jam: back balls out at half speed while the flywheel runs forward at "400 RPS" (= flat-out 12 V; 400 is the clamp ceiling, beyond physical free speed) so a ball pinched against the wheel ejects out the shooter instead of being trapped.

### W20. indexerDefault has no `addRequirements` — indexerDefault.java:16-22
```java
public indexerDefault(Indexer indexerSubsystem, BooleanSupplier intakeOut) {
    this.m_indexer = indexerSubsystem;
    this.intakeOut = intakeOut;
}
```
No requirement is declared, yet it's installed via `indexer.setDefaultCommand(...)` (RobotContainer.java:97). WPILib requires default commands to require their subsystem — this should throw at robot init. Either this code path has never run as committed, or scheduling is silently broken. **Must be resolved with the team.**

### W21. Vision stale-distance hold — Vision.java:36-38, 76-80
```java
if (!freshThisCycle || latestResult == null || !latestResult.hasTargets()) {
    distanceToHub = lastGoodDistance;
    distanceToPassingTag = lastGoodPassingDistance;
    return;
}
```
On tag loss, distances hold their last good value (so flywheel RPM doesn't spool down mid-volley during brief occlusion). Yaw fields freeze too, but consumers gate yaw on freshness flags.

### W22. Sticky tag tracking — Vision.java:86-96
First pass prefers the already-tracked tag even when another visible tag has lower ambiguity. Prevents aim twitch when the "best" tag flips between two hub faces (each face maps to hub center via a different lateral offset, so switching causes a step change in yaw/distance).

### W23. Ambiguity gate effectively disabled — VisionConstants.java:33
```java
public static final double MAX_POSE_AMBIGUITY = 1.0;//0.15;
```
Ambiguity is bounded [0,1]; a 1.0 threshold rejects nothing. The tuned 0.15 is commented out — loosened during debugging. A deliberately-disabled safety filter; still used as the acquisition tiebreaker.

### W24. Universal −0.2 m lateral fudge on every tag→hub vector — VisionConstants.java:149-169
```java
Map.entry( 2, hubVec( 0.000-0.2)),            // +Y face, centered
Map.entry( 3, hubVec(+HUB_LATERAL_OFFSET_METERS-0.2)),
...
Map.entry(27, hubVec(+HUB_LATERAL_OFFSET_METERS-0.2))  // -Y face, offset WAS POSITIVE BEFORE
```
All 16 hub entries subtract 0.2 m from the geometric lateral offset — an empirical aim calibration (likely compensating a camera-on-turret lateral mount offset) applied uniformly instead of via a proper camera transform. Hand-tuned; sacred. Also: X is always −0.604 (hub center is behind the tag face), Z ignored (distance is horizontal-plane only).

### W25. Passing yaw aims along the tag's surface normal, not at the tag — Vision.java:146-157
```java
Rotation3d tagToCamRot = cameraToTag.getRotation().unaryMinus();
Translation3d tagNormalInCam = new Translation3d(0, 0, 1).rotateBy(tagToCamRot);
yawToPassingTagDeg = -Math.toDegrees(Math.atan2(-tagNormalInCam.getY(), -tagNormalInCam.getX()));
```
For passing, the turret aligns with the direction the tag *faces* — lobbing the pass into the zone in front of the tag rather than at the tag itself. The negations convert tag-normal-toward-camera into camera-into-zone and match the hub yaw sign convention.

### W26. Hub yaw sign flip vs WPILib convention — Vision.java:144-145
```java
yawToHubDeg = -Math.toDegrees(Math.atan2(toHub.getY(), toHub.getX()));
```
Negated so positive = "target to the right," matching PhotonVision's `getYaw()` screen convention (which the BENCH_TEST path uses raw). Keeps both paths sign-consistent for the turret controller. `rawDeg` is logged alongside for comparison.

### W27. "FIX n" comment trail — Vision.java:63, 112, 119, 150, 165
Five numbered comments document a recent bug-fix pass (latched flags, NPE fall-through, inconsistent tag ids, passing distance clobbering hub distance, unwritten trackedPassingTagId). The passing path is new and was recently buggy — treat with suspicion.

### W28. Alliance handling: per-call in Vision, cached-forever in calculator — VisionConstants.java:88-91 / TurretAimingCalculator.java:89-105
Vision queries `DriverStation.getAlliance().orElse(Blue)` every periodic call; the calculator caches it until `clearAllianceCache()` (called in aim-command initialize). Tag classification precedence (hub checked before passing) is load-bearing because the ID sets overlap (3,4,9,10,19,20,25,26 appear in two sets).

### W29. Hood braceless-if escapes its null guard — RobotContainer.java:198-205
```java
if (hood != null) 

        operator.x().whileTrue(hood.run(...));
        operator.y().whileTrue(hood.run(...));   // NOT guarded
        operator.b().whileTrue(hood.run(...));   // NOT guarded
```
Only the X binding is guarded; Y and B would NPE at construction if `Gen.enableHood` were false. Masked today because all flags are true. Indentation lies.

### W30. Public static mutable globals — RobotContainer.java:60, 72
```java
public static CommandXboxController operator = ...;
public static TurretAutoAimCommand visionAutoAim;
```
The aim command doubles as a robot-wide state estimator service: it is simultaneously the turret default command AND the supplier source (`getDistanceToHub`, `isAimed`, `currentlyPassing`, `getCalculator`) for ShootCommand/AutoShootCommand.

### W31. Deferred ShootCommand with partial requirements — RobotContainer.java:148-156
`Commands.defer(..., Set.of(indexer, hood, shooter))` — drivetrain and turret are passed in but deliberately not required, so the drive default and turret auto-aim keep running while shooting. The fire button therefore *composes with* tracking instead of replacing it.

### W32. Hood default command is a sag-follow, not a hold — RobotContainer.java:91-94
```java
hood.setDefaultCommand(hood.run(() -> hood.setHoodAngle(Rotations.of(hood.getHoodPosition()))));
```
Re-samples position every loop and servos to it: if the hood sags, the setpoint sags with it ("track current position", not "hold position at interrupt").

### W33. HootAutoReplay always in the main loop — Robot.java:51-53, 75
`new HootAutoReplay().withTimestampReplay().withJoystickReplay()` updated before the scheduler every loop — enables deterministic .hoot log replay; no-op on a real field.

### W34. MATCH_MODE telemetry flag — Robot.java:47
`MATCH_MODE = false` → Epilogue publishes DEBUG+CRITICAL to NT. Match config is a manual flag flip to CRITICAL-only.

### W35. CANivore bus name has a double space — TunerConstants.java:100
```java
public static final CANBus kCANBus = new CANBus("Persian  Canivore", "./logs/example.hoot");
```
"Persian  Canivore" (two spaces) must match the name programmed into the CANivore exactly. Mechanisms live on a second bus `new CANBus("mechanisms")` (Gen.java:13). Classic single-space-typo trap for any future code.

### W36. Constants contradictions (all values are LAW; the comments are wrong)
- `SHOOTER_CONFIG.Voltage.PeakForwardVoltage = 12.0; // cap at 10V...` — value 12, comment 10 (TurretConstants.java:72). PeakReverse = 0.0 ("never apply reverse voltage — let friction stop it").
- `TURRET_POS_RIGHT = 0.25; // +18° CW (forward limit)` — 0.25 rot = +90°, and the forward limit is 0.39 (TurretConstants.java:166).
- Turret limit comments (L30-31) cite two older value sets; code says −0.73/+0.39 — neither comment matches.
- `TURRET_GEAR_RATIO = 122.0/24.0;// ??/18 //5.08` — tooth counts never confirmed; 5.0833 vs empirical 5.08 (TurretConstants.java:22).
- Hood kI=50 with kP=100 on a 0.08-rot mechanism, no visible anti-windup (TurretConstants.java:127-128).
- kS=0.15/kV=0.12/kP=0.3 copy-pasted verbatim across roller, all 3 indexers, and the shooter (160 A flywheel with 60 A-wheel gains).
- CAN IDs are declared twice (CANID enum + each constants file); values currently agree.
- `INTAKE_ENCODER_OFFSET = 0.0; // TODO: set magnet offset`; `IntakeCurrentSpike = 20` unused (vestigial ball-detection idea — there is NO game-piece sensing anywhere on this robot).
- One-sided shooter clamp: `Math.min(x, MAX_SHOOTER_RPS)` — negative requests pass through (Shooter.java:60).

### W37. Turret zero binding has no requirement — RobotContainer.java:171-174
Operator RB calls `turret.getTurretMotor().setPosition(0)` via a requirement-less `runOnce`, redefining the turret coordinate frame while the auto-aim default keeps running.

### W38. PathPlanner deploy quirks (deploy/pathplanner/)
- `" Manual Depot to Outpost.path"` has a **leading space** in the filename and is referenced with the leading space by `Intake Out Depot_Outpost.auto` — works only by exact-match accident.
- `Blue 2 Depot_Outpost.auto` starts with `Blue 2 to Depot` whose start pose is mid-field (1.87, 4.0) with `resetOdom: true` — odometry teleports mid-field; siblings start from wall poses. Likely a bug.
- `Blue 3 to Depot.path` is the only path with `useDefaultConstraints:false` and a **global** maxVel of 1.0 m/s (plus a redundant 1.0 zone) — entire path crawls; siblings slow only the approach.
- Two different "Blue 1" start poses ((3.6,1.9) vs (3.58,0.75)); "Center" end poses differ ((2.5,4.2) vs (2.5,4.0)).
- `deadline{ wait 8.0, shoot }` groups time-box shooting to 8 s.
- All paths face 180° (intake-first) except orphans; everything is Blue-side, red via AutoBuilder alliance flip.
- Orphans: 5 paths (3 iterations of "Blue 1 to Neutral Pass 1", "Nuetral to Shoot" [typo], "New New Path"), 3 empty auto folders, empty "New Folder".
- settings.json: default constraints 3.0 m/s / 3.0 m/s² / 540°/s / 720°/s; mass 49.895 kg; MOI 6.883; max module speed 5.44 (vs TunerConstants 5.85 — second source of truth via `RobotConfig.fromGUISettings()`).

### W39. Translation SysId logs to WPILOG while steer/rotation log to SignalLogger — CommandSwerveDrivetrain.java:69-79
Deliberate per comment ("No custom state logger — uses WPILib DataLog (WPILOG) by default") — a translation characterization session splits data across two log formats.

### W40. `visionAutoAim` constructed even when vision is null — RobotContainer.java:80-89
Guard checks only `turret != null`; a null vision is passed in (the command null-checks internally). With vision off it is never installed as default but still exists as a supplier source.

---

## 3. Complete controls map

**Driver — Xbox port 0** (`joystick`, Gen.driverPort=0)

| Input | Type | Action |
|---|---|---|
| Left stick Y/X | default | Field-centric translate, −Y/−X × 5.85 m/s, 5% deadband (in SwerveRequest), open-loop |
| Right stick X | default | Rotate, −X × 4.712 rad/s, 10% deadband |
| A | whileTrue | X-stance brake (SwerveDriveBrake) |
| LB | onTrue | `seedFieldCentric` (re-zero field heading) |
| B | whileTrue | **COMMENTED OUT** — CalibrationShootCommand ("uncomment for interp table calibration sessions only") |

No slow/turbo modes, no slew limiters, no input squaring, no alliance-based inversion, no rumble.

**Operator — Xbox port 1** (`operator`, public static, Gen.operatorPort=1)

| Input | Type | Action |
|---|---|---|
| A | toggleOnTrue | IntakeOutCommand (deploy+rollers; 2nd press: stop+restow) |
| RT | whileTrue | Deferred ShootCommand (vision mode; reqs {indexer,hood,shooter}) |
| LT | whileTrue | Vertical indexer at 35 RPS (startEnd) |
| RB | onTrue | Re-zero turret at current position (no requirement) |
| LB | (supplier) | Passing mode modifier — read inside TurretAutoAimCommand |
| Left stick click | (supplier) | Turret manual-override modifier — read inside TurretAutoAimCommand |
| Right stick X | (supplier) | Turret manual jog axis (±3 V, deadband 0.08) |
| Right stick click | whileTrue | reverseIndexers (all stages −50%, shooter forward max) |
| POV up/down/left/right | whileTrue | Turret presets 0.0 / −0.500 / −0.250 / 0.25 rot |
| X / Y / B | whileTrue | Hood presets 0.005 / 0.040 / 0.070 rot (Y and B escape the null guard, W29) |

**Default commands:** drivetrain → field-centric drive; turret → TurretAutoAimCommand (if turret && vision); hood → sag-follow hold; indexer → indexerDefault (vertical 26.25 RPS when intake out; if intake && indexer). Intake, shooter, vision: none.

---

## 4. Autonomous

**Wiring:** `drivetrain.configurePathPlanner()` (AutoBuilder, translation PID 10/0/0, rotation 7/0/0, `RobotConfig.fromGUISettings()`, alliance flip when Red) → `registerNamedCommands()` → `autoChooser = AutoBuilder.buildAutoChooser()` → `SmartDashboard.putData("Auto Chooser", ...)`. `getAutonomousCommand()` returns chooser selection; scheduled in autonomousInit, canceled in teleopInit. `PathPlannerPath` never used directly in Java.

**Named commands registered (RobotContainer.java:211-259, all null-guarded):**

| String | Maps to | Used by autos? |
|---|---|---|
| `intakeOut` | IntakeOutCommand | YES (also path event markers `DeployIntakeEvent`) |
| `intakeIn` | IntakeInCommand | no (registered only) |
| `shoot` | ShootCommand (passing pinned false) | YES |
| `stopAll` | sequence(stop shooter, stop indexer) | no |
| `autoShoot` | AutoShootCommand | YES |
| `feedIndexers` | runEnd all 3 indexers (35/35/100), stop on end | YES |
| `reverseIndexer` | reverseIndexers — **uses `shooter` without null guard inside `if (indexer != null)`** | no |

All four used strings resolve; no missing registrations.

**Autos (17, all `resetOdom: true`, all Blue-side):** Blue {1,2,3} × {Center Shoot, Depot_Center, Depot_Outpost, Outpost_Center, Outpost_Depot} + 2 "Intake Out" manual autos. Patterns: Center Shoot = drive+`shoot`; Depot_Center (1/3) = parallel `autoShoot` + drive + `feedIndexers`; Depot_Outpost = drive → wait 3 → drive → deadline{wait 8, `shoot`}; Outpost_Center = parallel `autoShoot` + waits + `feedIndexers`. Full sequence table in section 2/W38 notes; per-auto detail available from the deploy tree.

**Paths (21):** 16 live, 5 orphans. Event marker `DeployIntakeEvent`→`intakeOut` on 7 paths. Constraint zones slow Depot approaches to 1.0 m/s; `Blue 3 to Depot` is globally 1.0 m/s (W38).

---

## 5. Dashboard I/O classification

**INPUT (must survive):**

| Key | What | Where |
|---|---|---|
| `SmartDashboard/Auto Chooser` | SendableChooser from AutoBuilder | RobotContainer.java:109-110 |
| `/Calibration/HoodAngleRot` (default 0.04) | calibration hood input | CalibrationShootCommand.java:55 |
| `/Calibration/ShooterRPS` (default 40.0) | calibration RPS input | CalibrationShootCommand.java:58 |
| `Robot.MATCH_MODE` (code constant, not NT) | telemetry verbosity switch — operational input | Robot.java:47 |

**OUTPUT (candidates to strip/replace in design phase):**
- Epilogue `@Logged` trees for Robot, RobotContainer, all subsystems and the aim command. CRITICAL fields: turret position/setpoint/error/wrap flags/soft-limit faults, aim state machine (`state`, `distanceToHubMeters`, `visionTxDeg`, `visionActive`, `trackedTagId`, target/current/error, `lastGyroYawDeg`, `omega`), shooter setpoint/velocity, hood position, `intakeIsOut`, `overtravelRecovery`, all 12 Vision fields. DEBUG fields: currents, voltages, velocities, CANcoder absolutes.
- Telemetry.java (CTRE template): NT `DriveState/*` (Pose, Speeds, ModuleStates, ModuleTargets, ModulePositions, Timestamp, OdometryFrequency), `Pose/robotPose` (Field2d emulation), `MiscTelemetry/BatteryVoltage`, `SmartDashboard/Module 0..3` (Mechanism2d); mirrored to SignalLogger.
- `/Calibration/DistanceMeters` — live vision distance echo during calibration.
- SignalLogger SysId keys (`SysIdSteer_State`, `SysIdRotation_State`, `Rotational_Rate`).
- Console prints: turret boot config readback, config-apply failures, `[TURRET] Zeroed...`, `[TELEMETRY] MATCH_MODE=...`.

---

## 6. Cross-subsystem interaction map

```
Drivetrain (Pigeon2 yaw) ──────────► TurretAutoAimCommand (gyro FF, every loop)
Drivetrain (getState().Speeds) ────► TurretAutoAimCommand (omega — logged, compensation stubbed W4)
Drivetrain (getSwerveX/YSpeed…
            actually getState().Speeds) ► ShootCommand (dead shoot-while-moving math W11)
Vision ────────────────────────────► TurretAutoAimCommand (tx, distance, freshness, tag ids; hub + passing)
TurretAutoAimCommand (public static) ► ShootCommand / AutoShootCommand (distance, isAimed, currentlyPassing, calculator)
Intake.isOut() ────────────────────► indexerDefault (vertical feed gating)
Shooter.atShooterSetpoint() ───────► ShootCommand (indexer feed gating)
reverseIndexers ───────────────────► commands BOTH Indexer and Shooter
Subsystem motor getters ───────────► "FaultMonitor registration" (FaultMonitor itself NOT FOUND in repo)
DriverStation alliance ────────────► Vision tag sets (per-call), calculator hub center (cached), operator perspective, PathPlanner flip
```

**Explicit findings on robot state influencing the drivetrain:**
1. **Nothing influences the drivetrain.** No auto-aim drive override, no heading snap, no robot-state-dependent speed scaling. The drive default command is pure manual; the only non-manual drive sources are PathPlanner (auto), the A-button brake, LB heading reseed, and the disabled Idle request.
2. **Vision never reaches drivetrain pose.** `addVisionMeasurement` overrides exist (generated/stock, with fpga-time conversion) but have zero callers. Pose = odometry only. The AprilTag field layout JSON is deployed but never loaded.
3. Influence flows the other way: drivetrain yaw/omega feed the turret (W3, W4), and chassis speeds feed dead math in ShootCommand (W11).
4. For the refactor: the future Drive adapter needs to expose, at minimum, what is consumed today — Pigeon2 yaw (or heading), chassis speeds, pose (unused today), and `addVisionMeasurement` (unused today).

**Construction wiring (RobotContainer.java:64-110):** drivetrain via `TunerConstants.createDrivetrain()`; Vision no-arg; Intake/Indexer/Turret/Hood/Shooter on `Gen.mechanismBus` ("mechanisms"); every mechanism nullable behind `Gen.enable*` flags (all currently true); no subsystem holds a reference to another — all coupling is via commands and suppliers.
