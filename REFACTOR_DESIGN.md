# REFACTOR_DESIGN.md — Rebuilt-2026 Phase 2

Design for the full rewrite. Inputs: FUNCTIONALITY_INVENTORY.md (signed off), its Phase 1 verification decisions, and the architecture skill. No robot code changes in this phase. Every inventory row maps to a destination in §10; zero unmapped rows is the acceptance criterion.

Design-affecting decisions from Phase 1 verification: D1 shoot-while-moving rebuilt behind a toggle (default OFF); D2 lose-aim feed bug fixed; D3 untested commands treated as intent, not verified behavior; D5 vision ambiguity gate returns at 0.3; D8 delete LED/Climb/runRoller/manualIndexer/orphan paths (SysId: see §5); D11 full vision pose estimation, single-camera robust; D12 AutoShootCommand deleted; D13 FaultMonitor leftovers deleted.

---

## 1. State and goal enums

### 1.1 RobotState (the scoring chain)

Extracted from implicit states in the old code: TurretAutoAimCommand's logged `state` ∈ {TRACKING, MANUAL, PASSING}, ShootCommand's spin-up gate (W13), and reverseIndexers.

```java
public enum RobotState {
  IDLE,             // turret tracks hub (old TRACKING — tracking IS idle on this robot), shooter off, hood holds
  INTAKING,         // intake deployed + rollers running, otherwise IDLE behavior + vertical feed at 26.25 RPS (W18)
  SPINNING_UP,      // SHOOT requested: shooter+hood track interp tables, feed gate closed
  SHOOTING,         // aimed && at speed: all indexer stages feed
  CLEARING,         // [ATOMIC] shot released — shooter stops, all indexers reverse FULL speed for kClearingSeconds (2.0 s)
  PASS_AIMING,      // PASS modifier: turret aims along passing-tag normal, shooter off
  PASS_SPINNING_UP, // SHOOT+pass: hood 0.067, shooter 95 RPS, feed gate closed
  PASSING,          // at speed: feed
  UNJAMMING,        // indexers reversed 50%, shooter forward at max
  MANUAL            // superstructure stops commanding; operator drives mechanisms directly
}
```

**INTAKING** (team revision, Phase 2 review): intaking-without-shooting is a real, visible robot mode and gets its own state. Intake-while-shooting concurrency is preserved because the intake *deployment* is still carried by the `intakeDeployed` condition: from INTAKING, requesting SHOOT goes to SPINNING_UP with the intake staying deployed, and when shooting ends the machine returns to INTAKING (not IDLE) if the intake is still out. So INTAKING is where the robot rests while collecting; the condition is how deployment coexists with the shoot states. The Intake mechanism's nested machine (§3.5) still owns the deploy/stow *sequencing*.

**CLEARING** (team revision, Phase 2 review + sign-off amendment — the one ATOMIC transition): when the operator releases the shoot trigger mid-volley, balls already committed to the feed path used to be stopped instantly, leaving them pinched against the spinning-down flywheel — the jam that made reverseIndexers necessary. CLEARING stops the shooter and immediately reverses ALL indexer stages at full speed (−35/−35/−100 RPS) for `kClearingSeconds` (tunable, default 2.0 s — team-specified), backing every committed ball away from the flywheel; no balls are wasted. It is atomic against IDLE/PASS goal changes (it completes), SHOOT re-request exits it immediately, and UNJAM and MANUAL preempt it instantly — recovery and manual override are never blockable.

**Deliberately NOT states:**
- *Calibration* is not a state. CalibrationShootCommand bypasses goal logic exactly like manual mode; it stays a standalone command requiring the same mechanisms (preserved workflow, binding stays commented).
- No HOMING/CLIMB/EJECT states — the robot boot-zeroes (no homing sequence), climb is deleted (D8), and eject IS unjam.

### 1.2 Goal

```java
public enum Goal {
  IDLE,    // default — return here whenever nothing is requested
  INTAKE,  // collect game pieces; the only goal that COMPOSES with the others (toggle semantics)
  SHOOT,   // score into the hub (or lob, when pass modifier held)
  PASS,    // aim for a cross-field pass without firing
  UNJAM    // back balls out / eject pinched ball through the flywheel
}
```

No TRACK goal: hub tracking is what IDLE does (the old turret default command always ran). No CLIMB: deleted per D8.

**Composition rule:** `INTAKE` is a toggle on an orthogonal axis (`Superstructure.toggleIntake()`); IDLE/SHOOT/PASS/UNJAM are mutually exclusive on the scoring axis (`Superstructure.setGoal(Goal)`, computed every loop from bindings). This mirrors the old code exactly, where intake was an independent toggle command and `isPassing` was a supplier into the aim and shoot commands rather than a separate command.

**Goal derivation from operator inputs (every loop):**
`UNJAM` if right-stick-click held; else `SHOOT` if RT held; else `PASS` if LB held; else `IDLE`. LB while RT selects the pass chain via the `passSelected` condition (matches old `isPassing` supplier semantics). Manual mode is NOT a goal — it is a mode toggle that suspends the goal machine (§7).

### 1.3 Conditions (input to pure transition logic)

```java
public record Conditions(
    boolean passSelected,        // LB held (old isPassing supplier)
    boolean aimed,               // |last vision tx| < 3° and fresh-frame seen since enable (fixes W10 false-ready)
    boolean atShooterSpeed,      // |velocity − setpoint| < 1.67 RPS
    boolean shooterCommandedNonZero, // W12 fix: a 0-RPS setpoint can never satisfy the feed gate
    boolean intakeDeployed,      // Intake nested machine reports deployed past threshold (W18 semantics)
    boolean clearingElapsed) {}  // CLEARING timer expired (timer lives in Superstructure; Transitions stays pure)
```

Hood-at-setpoint is deliberately absent: the old teleop feed gate ignored hood readiness (W13) and that is preserved.

---

## 2. Transition table

`Transitions.next(RobotState, Goal, Conditions) -> RobotState` — pure, hardware-free, exhaustive arrow switch.

`REST` below means the no-request resting state: `intakeDeployed ? INTAKING : IDLE`. IDLE and INTAKING share all outbound transitions (the intake axis never blocks a scoring request).

| # | From | Goal + conditions | → To | Atomic? |
|---|---|---|---|---|
| T0 | IDLE ↔ INTAKING | intakeDeployed changed (A toggle) | INTAKING / IDLE | no |
| T1 | IDLE, INTAKING | SHOOT, !passSelected | SPINNING_UP | no |
| T2 | IDLE, INTAKING | SHOOT, passSelected | PASS_SPINNING_UP | no |
| T3 | IDLE, INTAKING | PASS | PASS_AIMING | no |
| T4 | IDLE, INTAKING | UNJAM | UNJAMMING | no |
| T5 | SPINNING_UP | SHOOT, aimed && atShooterSpeed && shooterCommandedNonZero, !passSelected | SHOOTING | no |
| T6 | SPINNING_UP | SHOOT, passSelected | PASS_SPINNING_UP | no |
| T7 | SPINNING_UP | goal ≠ SHOOT | per goal: REST / PASS_AIMING / UNJAMMING (no balls committed yet — no follow-through) | no |
| T8 | SHOOTING | SHOOT, !(aimed && atShooterSpeed) | SPINNING_UP (anti-dribble re-close, W13; aim-loss path is the W12 fix) | no |
| T9 | SHOOTING | SHOOT, passSelected | PASS_SPINNING_UP | no |
| T10 | SHOOTING | goal ∈ {IDLE, PASS} | **CLEARING** (balls are committed — back them out first) | **YES** |
| T10u | SHOOTING | UNJAM | UNJAMMING (recovery always preempts) | no |
| T11 | PASS_AIMING | SHOOT (passSelected implied by LB) | PASS_SPINNING_UP | no |
| T12 | PASS_AIMING | IDLE | REST | no |
| T13 | PASS_AIMING | UNJAM | UNJAMMING | no |
| T14 | PASS_SPINNING_UP | SHOOT, atShooterSpeed | PASSING | no |
| T15 | PASS_SPINNING_UP | SHOOT, !passSelected | SPINNING_UP (LB released mid-spin) | no |
| T16 | PASS_SPINNING_UP | goal ≠ SHOOT | per goal: REST / PASS_AIMING / UNJAMMING | no |
| T17 | PASSING | SHOOT, !atShooterSpeed | PASS_SPINNING_UP | no |
| T18 | PASSING | goal ∈ {IDLE, PASS} | **CLEARING** | **YES** |
| T18u | PASSING | UNJAM | UNJAMMING | no |
| T19 | UNJAMMING | goal ≠ UNJAM | REST (or PASS_AIMING if LB held) | no |
| T22 | CLEARING | clearingElapsed | REST (or PASS_AIMING if LB held) | no |
| T23 | CLEARING | SHOOT re-requested | SPINNING_UP / PASS_SPINNING_UP per passSelected | no |
| T24 | CLEARING | UNJAM | UNJAMMING (preempts the atomic clear — recovery wins) | no |
| T20 | any | manual toggle pressed | MANUAL (mode flag checked before goal logic, see §7; preempts everything incl. CLEARING) | no |
| T21 | MANUAL | manual toggle pressed | IDLE via re-sync (§7) | no |

Illegal by construction (the table is exhaustive; anything not listed resolves to the goal's entry state or stays put): SHOOTING cannot be entered except through SPINNING_UP with the full gate satisfied; PASSING cannot be entered without `atShooterSpeed`; CLEARING cannot be entered except from SHOOTING/PASSING; UNJAMMING cannot be entered from MANUAL (manual suspends the machine entirely).

**Atomic transitions: exactly one — the CLEARING back-out (T10/T18).** Team decision at Phase 2 review, amended at sign-off: stopping the feed instantly mid-volley pinches committed balls against the spinning-down flywheel, which is precisely the jam reverseIndexers exists to clear. CLEARING stops the shooter and reverses all three indexer stages at FULL speed (−35/−35/−100 RPS) for `kClearingSeconds` (default 2.0 s, tunable), backing the committed balls out — no balls wasted. Atomicity is scoped: only IDLE/PASS goal changes are deferred until the clear completes; SHOOT re-request (T23), UNJAM (T24) and MANUAL (T20) all exit/preempt immediately — re-engagement, recovery and the manual override are never blockable (non-negotiable #5).

**Return-to-rest default:** the goal supplier produces `IDLE` whenever no operator input is held, and every state has a goal-exit row, so the machine returns to REST (= INTAKING if the intake toggle is on, else IDLE) within one loop of release — except through the bounded CLEARING back-out. IDLE behavior: turret hub-tracks (visual servo + gyro FF), shooter stopped, hood holds via capture-once setpoint (replaces sag-follow, see W32 note in §10), all indexers stopped. INTAKING behavior: IDLE + intake deployed/rolling + vertical indexer at 26.25 RPS (W18 verbatim).

**W12 fix (decision D2):** two layers — (a) when aim is lost in SPINNING_UP/SHOOTING the shooter keeps tracking the interp-table speed at the stale-held distance instead of being commanded 0; (b) `shooterCommandedNonZero` is in the feed gate, so a zero setpoint can never open it regardless of (a).

---

## 3. Per-mechanism contracts

Common shape for every mechanism: inputs read once per loop in `periodic()` via `io.updateInputs(inputs)`; logic reads only the inputs record; config-apply with the existing 5×-retry pattern lives in the real IO; all tuned values copied verbatim into per-subsystem constants files. Sim IO ships with real IO in the same PR. FaultMonitor motor getters are deleted (D13).

### 3.1 Shooter

- Setpoints: `setVelocity(AngularVelocity)` (clamps **both** bounds: [0, MAX_SHOOTER_RPS] — closes the W36 negative-passthrough edge; no caller ever passed negative), `stop()`.
- Predicates: `atSpeed()` (±1.67 RPS), `commandedNonZero()`.
- `ShooterIO`: `updateInputs(ShooterIOInputs)`, `setVelocity(double rps)`, `stop()`.
- `ShooterIOInputs` record: `velocityRPS, setpointRPS, supplyCurrentAmps, appliedVolts`.
- Sim: `FlywheelSim` (DCMotor.getKrakenX60(1), measured-equivalent MOI placeholder tuned until spin-up time matches video/log).
- Nested machine: **no** — pure velocity executor.

### 3.2 Hood

- Setpoints: `setAngle(Angle)` (clamp [0.0, 0.08] rot verbatim), `stop()`, `holdCurrentPosition()` (captures position ONCE then holds — see W32 mapping), `setVoltage(double)` (manual mode only).
- Predicates: `atSetpoint()` (±0.005 rot), `getPosition()`.
- `HoodIO`: `updateInputs`, `setTargetPosition(double rot)`, `setVoltage(double)`, `zeroAtCurrentPosition()`, `stop()`.
- `HoodIOInputs`: `positionRot, velocityRPS, setpointRot, supplyCurrentAmps, appliedVolts`.
- Sim: `DCMotorSim` with gear ratio 155/12 (old kG=0 — gravity was never compensated, so an arm sim would model physics the gains were never tuned for; DCMotorSim reproduces the tuned behavior).
- Nested machine: **no**. Boot-zero-at-bottom stays (confirmed procedure, D4).

### 3.3 Turret

- Setpoints: `setFieldTarget(...)` — no; the turret stays a *position executor*: `setTargetPosition(Angle)` returns the **actually-commanded post-wrap angle** (preserves W1+W2 contract), `setVoltage(double joystick × 3.0 V)` (manual), `zeroAtCurrentPosition()` (operator RB + boot), `stop()`.
- Predicates: `atSetpoint()` (±0.005 rot), `getPosition()`, `wrapped()` (last command wrapped — kept CRITICAL-logged).
- Wrap: `wrapAndClamp` becomes a **pure static function** `TurretWrap.apply(double rot, double fwdLimit, double revLimit)` — verbatim ±1.0-then-clamp logic, **unit-tested directly** (it is the highest-consequence pure logic in the robot).
- `TurretIO`: `updateInputs`, `setTargetPosition(double rot, double ffVolts)`, `setVoltage(double)`, `zeroAtCurrentPosition()`, `stop()`.
- `TurretIOInputs`: `positionRot, velocityRPS, setpointRot, supplyCurrentAmps, appliedVolts, cancoderAbsRot, fwdSoftLimitFault, revSoftLimitFault`.
- Sim: `DCMotorSim`, gear ratio 122/24; soft-limit faults simulated by clamping at ±limits.
- Nested machine: **no — and this is the deliberate call the inventory flags as "likely candidate."** The turret's apparent states (TRACKING/PASSING/MANUAL) are *robot-level aiming decisions*, not internal mechanism modes — they were implicit robot states all along and move into RobotState. The turret has no homing sequence (boot-zero) and wrap is stateless math per command. A nested machine here would duplicate the Superstructure. The aiming *logic* (visual servo + gyro FF + frame dedupe, W3–W7) lives in a `TurretAiming` helper owned by the Superstructure (pure where possible), not in the Turret subsystem — the turret must not know about vision or the drivetrain (mechanism contract: zero cross-subsystem references).

### 3.4 Indexer

- Setpoints: `setVelocity(double rps, Stage)` with `Stage { HORIZONTAL, VERTICAL, UPWARD }` (enum moves out of constants into the subsystem), `stop(Stage)`, `stopAll()`.
- Predicates: none needed (no sensors exist; velocity telemetry only).
- `IndexerIO`: `updateInputs`, `setVelocity(double rps, int stage)`, `stop(int stage)`.
- `IndexerIOInputs`: `horizontalVelocityRPS, verticalVelocityRPS, upwardVelocityRPS, setpoints[3], currents[3]`.
- Sim: 3× `DCMotorSim`.
- Nested machine: **no** — three dumb velocity wheels. All feed *decisions* (W13 gating, W18 idle feed, W19 unjam mix) belong to the Superstructure.

### 3.5 Intake — the one justified nested machine

- Setpoints: `requestDeploy()` / `requestStow()` (drive the nested machine), `setPivotVoltage(double)` + `setRollerSpeed(double)` (manual mode only), `stop()`.
- Predicates: `isDeployed()` (nested state == DEPLOYED_ROLLING), `isOut()` (pivot > stow + 5° — verbatim W18 semantics, feeds `Conditions.intakeDeployed`), `atSetpoint(Angle tol)`.
- Nested machine (justified in the class comment): `STOWED → DEPLOYING → DEPLOYED_ROLLING → STOWING → STOWED`. **Why yes:** the deploy sequencing is genuinely internal and temporal — rollers start only within 15° of out and then LATCH on (W17), stow must stop rollers first then travel, and over-travel recovery (W16) must keep running every loop regardless of what the robot-level machine wants. These are mechanism-protection behaviors with their own memory; micromanaging them from the Superstructure would leak pivot tolerances and latch flags upward. The nested machine makes zero robot-level decisions — it only sequences its own deploy/stow.
- Over-travel recovery runs in `periodic()` exactly as today (active when STOWING/STOWED and pivot < 0.14).
- `IntakeIO`: `updateInputs`, `setPivotTarget(double rot)`, `setPivotVoltage(double)`, `setRollerVelocity(double rps)`, `stopRoller()` (NeutralOut — coast, verbatim), `stop()`.
- `IntakeIOInputs`: `pivotPositionRot, pivotSetpointRot, pivotCurrentAmps, rollerVelocityRPS, rollerCurrentAmps, cancoderAbsRot`.
- Sim: pivot `SingleJointedArmSim` (gravity is exactly why over-travel recovery exists — the sim must reproduce the failure mode so the recovery is testable), roller `DCMotorSim`.

### 3.6 Vision

Two responsibilities, one camera, one subsystem: **targeting** (preserved verbatim in behavior) and **pose estimation** (new, D11).

- `VisionIO`: `updateInputs(VisionIOInputs)`. Real: `PhotonCamera("mugilanr")`, drain `getAllUnreadResults()`. Sim: `PhotonCamera` + photonlib `VisionSystemSim` fed from the simulated drive pose + simulated turret angle.
- `VisionIOInputs`: `connected, hasNewResult, timestampSeconds, latencyMs,` per-target arrays `targetIds[], yawDeg[], ambiguity[], camToTargetTransforms[]`, and `multitagPoseEstimate (Optional via flag + fields), multitagTagCount`.
- **Targeting pipeline** (logic in the subsystem, reading inputs only): sticky tag tracking (W22), hub-before-passing classification with the overlapping ID sets (W28), tag→hub vectors with the −0.2 fudges verbatim (W24, D6), hub yaw sign flip (W26), passing yaw from tag surface normal (W25), stale-distance hold per class (W21), exact-timestamp dedupe exposed as `isResultFresh()/getResultTimestamp()` for the aiming helper (W5). `MAX_POSE_AMBIGUITY = 0.3` (D5 — re-enabled, new value).
- Predicates/getters (same surface the aim logic consumes today): `isSeeingHubTag()`, `getYawToHubDeg()`, `getDistanceToHub()`, `getTrackedTagId()`, passing variants, `isResultFresh()`, `getResultTimestamp()`.
- **Pose estimation** (`VisionPoseEstimator`, owned by Vision, output to Drive adapter). Design goal per team review: **as simple as possible** — this is the documented photonlib pattern plus three if-statements, ~60 lines total, no custom math beyond one transform compose.
  - **The camera-on-turret transform is two constants and one line.** Constants in VisionConstants (placeholder values now, team measures later — tape measure, three numbers each):
    ```java
    // TODO(measure): turret azimuth axis in robot frame (x fwd, y left, z up from floor)
    public static final Transform3d ROBOT_TO_TURRET = new Transform3d(...);
    // TODO(measure): camera in turret frame WITH TURRET AT ZERO (the boot-zero straight-forward pose)
    public static final Transform3d TURRET_TO_CAMERA = new Transform3d(...);
    ```
    Per frame:
    ```java
    Transform3d robotToCamera = ROBOT_TO_TURRET
        .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0, 0, turret.getPosition().getRadians())))
        .plus(TURRET_TO_CAMERA);
    ```
    This is the exact pattern PhotonVision's own docs use for turret-mounted cameras (verified via Context7, photonvision simulation docs: "Update Camera Transform for Moving Mechanisms"). The *current* turret angle is used — no timestamp-aligned angle history. That would normally be an approximation error, so we make it a non-issue instead of compensating for it:
  - **Gate 1 — turret slew reject:** skip pose frames while `|turret velocity| > kMaxTurretSlewForPose` (tunable, ~0.25 RPS). When the turret is steady, "current angle" IS the angle at capture time. Cheaper and more honest than buffering angle history.
  - **Gate 2 — ambiguity:** single-tag frames with ambiguity > 0.3 are dropped (D5).
  - **Gate 3 — on-field:** estimated pose outside the field rectangle is dropped.
  - Estimator: `PhotonPoseEstimator` (field layout from the deployed `FRC2026_WELDED.json`), per result: `estimateCoprocMultiTagPose(result)` → if empty, `estimateLowestAmbiguityPose(result)` (the photonlib-recommended two-call pattern, verified via Context7). robotToCamera updated each frame before estimating.
  - Std devs: the standard photonlib two-tier scheme — fixed `kMultiTagStdDevs` and a higher `kSingleTagStdDevs`, scaled by average tag distance. Three tunable numbers, ~10 lines. No custom covariance math.
  - Output: `drive.addVisionMeasurement(pose2d, timestamp, stdDevs)` — finally giving the dead overrides a caller.
  - Honest risk, stated once: pose quality inherits turret boot-zero truth (W8). The RB re-zero button already exists as the field fix; conservative single-tag std devs bound the damage. No CANcoder fusion, no per-tag trust tables, no multi-camera scaffolding — none of it until a real match log says we need it.
- Nested machine: **no**.

### 3.7 ShotCalculator (util, pure)

Renamed from TurretAimingCalculator. Interp tables verbatim (4 points each, calibrated 2026-03-17), validity window 1.0–7.0 m, alliance-cached hub-center selection with explicit invalidation (W—calculator cache; cleared on enable). **Shoot-while-moving (D1):** `effectiveDistance(dist, chassisSpeeds, turretAngle)` — the closing-velocity math from ShootCommand:90-106 moved here, finished correctly (lookups take effective distance), behind `ShooterConstants.kShootWhileMovingEnabled` tunable toggle, **default false**. Unit-tested with synthetic speeds.

---

## 4. Drive adapter API

`subsystems/drive/Drive.java` wraps the generated drivetrain. Generated files end Phase 3 byte-identical to the **stock Tuner X template**: the three hand additions currently inside CommandSwerveDrivetrain (`getSwerveXSpeed/getSwerveYSpeed`, `configurePathPlanner()`, `@Logged`) move INTO the adapter, restoring the generated file to stock, which is then frozen (verified with git diff per the drivetrain policy). TunerConstants is untouched verbatim (including `"Persian  Canivore"`, D10). The stock template's SysId plumbing stays in the generated file (vendor template wins over the D8 cleanliness sweep — it is inert and unbound; deleting it would mean editing a frozen file); our unbound SysId *bindings* never existed, so nothing else to delete.

Surface — exactly what the inventory shows is consumed plus the two new consumers (pose estimation, shot calculator):

| Method | Serves | Replaces (inventory ref) |
|---|---|---|
| `Command teleopDriveCommand(DoubleSupplier x, y, omega)` | driver default | FieldCentric request, 5.85 m/s / 4.712 rad/s, 5%/10% deadbands, negated axes, OpenLoopVoltage — all verbatim (§1.1) |
| `Command brakeCommand()` | driver A | SwerveDriveBrake |
| `Command seedFieldCentricCommand()` | driver LB | seedFieldCentric |
| `Command idleWhileDisabledCommand()` | disabled trigger | SwerveRequest.Idle + ignoringDisable (W—disabled idle) |
| `double getContinuousYawDeg()` | turret gyro FF | `getPigeon2().getYaw()` — **same signal, same sign** (W3 is load-bearing); continuous (no ±180 wrap), which the delta integration requires |
| `ChassisSpeeds getRobotRelativeSpeeds()` | omega logging (W4 scaffolding), ShotCalculator (D1) | `getState().Speeds`, getSwerveX/YSpeed |
| `Pose2d getPose()` | pose telemetry, future auto-aim | `getState().Pose` |
| `Optional<Pose2d> samplePoseAt(double timestamp)` | vision latency comp | samplePoseAt override (null→Optional per style law) |
| `void addVisionMeasurement(Pose2d, double t, Matrix<N3,N1> σ)` | VisionPoseEstimator (NEW) | the dead overrides get their caller (D11) |
| `void configureAutoBuilder()` | RobotContainer init | configurePathPlanner() moved here verbatim (PID 10/0/0, 7/0/0, fromGUISettings, alliance flip, wheel-force FFs) — uses only public drivetrain API |
| `void registerTelemetry(Consumer<SwerveDriveState>)` | Telemetry hookup | pass-through |

Robot state influencing driving: **none today, none added** (inventory §6 finding 1). If auto-aim-while-driving ever arrives, it flows through this adapter per the drivetrain policy — the seam exists, nothing speculative is built.

Sim: CTRE `SimState` via the generated sim thread, untouched (per the amended simulation-stack policy). maple-sim is not adopted (its CTRE template requires touching the generated class).

---

## 5. Dashboard plan

All INPUT entries survive. OUTPUT default is STRIP from NT; keeps are justified. The logging *framework* stays deferred — Epilogue remains the incumbent mechanism behind the IO seam, with `MATCH_MODE` retained as the verbosity switch; everything stripped from NT remains available in log files at DEBUG importance.

**INPUT (all survive):**
| Entry | Disposition |
|---|---|
| `SmartDashboard/Auto Chooser` | keep (AutoBuilder chooser) |
| `/Calibration/HoodAngleRot`, `/Calibration/ShooterRPS` | keep (calibration workflow preserved) |
| `MATCH_MODE` flag | keep (Robot.java constant) |
| NEW: `Tuning/ShootWhileMoving` toggle | added per D1, default false |
| NEW: tuning flag for tunable numbers | per skill tunables pattern |

**OUTPUT — keep on NT (justified):**
| Entry | Why |
|---|---|
| Superstructure `state`, `goal` (replaces aim command `state`) | drive team + pit debugging; the single most diagnostic signal |
| Turret position/setpoint/`wrapped` + soft-limit faults | cable-chain protection; wrap events must be visible live (W1) |
| Shooter velocity/setpoint (`atSpeed` implicit) | "why isn't it firing" triage |
| Vision: `cameraConnected`, `seeingHubTag`, `distanceToHub`, tracked tag, NEW pose-accepted/rejected counters | single-camera robot; camera health is match-critical |
| Intake `isDeployed`/`overtravelRecovery` | W16 events indicate collisions; drive team visibility |
| `DriveState/Pose` + Field2d `Pose/robotPose` | pose estimation (D11) needs live field view for trust calibration |
| `MiscTelemetry/BatteryVoltage` | brownout triage |
| `/Calibration/DistanceMeters` | calibration workflow echo |

**OUTPUT — strip from NT (remain in logs at DEBUG):** all per-motor currents/voltages/velocities (turret/hood/shooter/intake/indexer DEBUG fields), CANcoder absolutes, `rawDeg`, `lastGyroYawDeg`, `omega`, `gyroFeedforwardRot`, aim-command duplicates of turret telemetry, `DriveState/ModuleStates/ModuleTargets/ModulePositions/Timestamp/OdometryFrequency` (hoot log keeps them), `SmartDashboard/Module 0..3` Mechanism2d widgets, SignalLogger SysId keys (inert), console boot printouts (turret config readback stays — it is a safety check, W9, console-only).

---

## 6. Test plan

Pure units: `Transitions` (the machine), `TurretWrap` (W1 math), `ShotCalculator` (tables + D1 effective distance). All hardware-free.

**TransitionsTest** (every legal transition + rejections + return-to-rest):
- `idleToIntakingOnToggle` + `intakingToIdleOnToggle` (T0), `intakingToSpinningUpKeepsIntakeDeployed` (T1 from INTAKING)
- `idleToSpinningUpOnShoot` (T1), `idleToPassSpinningUpWithPassSelected` (T2), `idleToPassAimingOnPass` (T3), `idleToUnjamming` (T4)
- `spinningUpHoldsUntilFullGate` (T5 negative: aimed only / atSpeed only / zero setpoint → stays SPINNING_UP — **the W12 regression test**)
- `spinningUpToShootingWhenGateSatisfied` (T5), `spinningUpReleaseSkipsClearing` (T7 — no balls committed, no clear)
- `shootingReclosesOnSpeedDroop` (T8 — W13 anti-dribble), `shootingReclosesOnAimLoss` (T8 — D2)
- `shootingReleaseEntersClearing` (T10), `passingReleaseEntersClearing` (T18)
- `clearingDefersIdleAndPassGoalsUntilElapsed` (**the atomic-completion-under-preemption test**: goal flaps IDLE/PASS mid-clear, state stays CLEARING until `clearingElapsed`)
- `clearingExitsToIdleOrIntakingPerIntakeToggle` (T22 → REST), `clearingReentersShootChain` (T23), `unjamPreemptsClearing` (T24), `manualPreemptsClearing` (T20 beats the atomic clear)
- `passChainRequiresOnlySpeed` (T14 — aimed NOT required in passing, preserved quirk)
- `passingReclosesOnSpeedDroop` (T17), `lbReleaseMidPassSpinFallsBackToHubChain` (T15)
- `everyStateReturnsToRestWhenGoalIdle` (parameterized over all states; CLEARING returns only after elapsed)
- `shootingUnreachableFromIdleDirectly`, `passingUnreachableWithoutSpeed`, `clearingUnreachableExceptFromShootingOrPassing`, `unjammingUnreachableFromManual` (illegal-transition rejections)
- `goalPreemptionWinsImmediately` (SHOOT→UNJAM mid-volley replans same tick)
- `manualEntryFromEveryState` + `manualExitLandsAtIdle` (T20/T21, tested at the Superstructure mode layer)

**TurretWrapTest:** `wrapAddsOneRotationBelowReverseLimit`, `wrapSubtractsOneAboveForwardLimit`, `wrapResultStillOutOfRangeClamps` (the <720° travel case), `inRangePassesThrough`, `wrappedFlagReportsCorrectly`, `valuesMoreThanOneRotationOutClampOnly` (documents the single-correction behavior).

**ShotCalculatorTest:** table endpoints clamp (W—1.55/4.62 vs 1.0–7.0 validity), known midpoints interpolate, alliance cache selects correct hub + invalidation works, `effectiveDistanceIdentityWhenToggleOff` (D1 default), `effectiveDistanceShortensWhenClosing`.

**IntakeSequencingTest** (nested machine, pure part): `rollersStartOnlyWithin15Degrees`, `rollerLatchSurvivesPivotBounce` (W17), `stowStopsRollersBeforeTravel`, `overtravelRecoveryOnlyWhenStowing` (W16).

CI: all of the above + `./gradlew build` + Spotless, per non-negotiable #9. Sim demonstration for each mechanism PR per #3.

---

## 7. Manual mode design

- **Toggle:** operator **Start** button (unused today; nothing is displaced). Toggles `manualMode` in the Superstructure; CRITICAL-logged; LED-free robot, so the dashboard `state=MANUAL` is the indicator.
- **Entry:** allowed from ANY state including faulted/vision-dead (the toggle is checked before goal logic; no condition can block it). Superstructure immediately stops commanding all mechanisms (one `stopAllForManual()` that stops shooter and indexers, freezes hood and turret targets at current positions).
- **Manual bindings (direct mechanism methods, zero Superstructure/vision involvement):**
  | Input | Drives |
  |---|---|
  | Right stick X | `turret.setVoltage(x)` (±3 V, 0.08 deadband — verbatim W7 feel) |
  | Right stick Y | `hood.setVoltage(y × kHoodManualVolts)` (new; soft limits still protect, firmware-side) |
  | RT | `shooter.setVelocity(SHOOTER_RPS = 55)` (the existing unused constant becomes the manual preset) |
  | LT | vertical indexer 35 RPS (same as today) |
  | Right stick click | unjam mix direct (indexers −50%, shooter max — verbatim W19) |
  | A | intake deploy/stow toggle (nested machine — mechanism-level, sensor-free, safe in manual) |
  | POV | turret position presets (0 / −0.5 / −0.25 / 0.25 — these already exist as direct setpoints) |
  | X/Y/B | hood presets 0.005 / 0.040 / 0.070 (verbatim) |
- Works sensors-dead: every manual path is voltage or velocity control on the rotor sensor; nothing reads vision; turret/hood firmware soft limits remain the safety net (exactly why W7's open-loop manual was already safe).
- **Exit re-sync (to IDLE, never to pre-manual state):** turret target rebased to current position (the W7 pattern, now the exit contract), hood `holdCurrentPosition()` (capture-once), shooter stopped, indexers stopped, intake nested machine keeps whatever deploy state it is in (it is self-consistent), state := IDLE, goal := IDLE. Vision/alliance caches cleared (calculator cache invalidation, W28).
- No PR may break this path (non-negotiable #5); `manualEntryFromEveryState` test enforces reachability.

---

## 8. Rewrite order for Phase 3

Each step ends with a green build, passing tests, and a sim demonstration. Mechanisms before Superstructure (it consumes their predicates); vision before Superstructure (conditions need it); drive adapter early (turret FF and pose estimation both hang off it).

1. **Skeleton + deletions.** New folder structure; constants migrated VERBATIM into per-subsystem files (single source for CAN IDs — kill the duplication); delete LED, Climb (all references incl. CANID entry), runRoller, manualIndexer, AutoShootCommand, FaultMonitor getters, orphan paths/empty folders, unused imports (D8/D12/D13). Lowest risk, biggest noise reduction; everything after diffs cleanly.
2. **Shooter, Hood, Indexer** (simplest mechanisms) — IO real+sim, subsystem logic, tunables. Proves the IO pattern on easy ground.
3. **Intake** — IO + nested machine + sequencing tests + arm sim (over-travel recovery reproduced in sim).
4. **Turret** — IO + `TurretWrap` pure function + tests + sim. Highest-consequence mechanism gets the pattern only after it is proven.
5. **Drive adapter** — move hand additions out, restore generated file to stock (git diff verification), AutoBuilder config in adapter, teleop bindings against adapter factories. Robot drives in sim.
6. **Vision** — IO + targeting parity (behavioral diff against inventory quotes) + `VisionSystemSim`; then pose estimation with gates/std-devs; wire `addVisionMeasurement`. Includes the turret→camera transform measurement task on hardware.
7. **Superstructure** — RobotState/Goal/Conditions/Transitions + full test suite; TurretAiming helper (W3–W7 verbatim semantics); manual mode; goal bindings. End-to-end sim: request goals, watch states advance.
8. **Autos** — named commands re-registered against Superstructure goal requests; replace the deleted AutoShootCommand pattern (`autoShoot`+`feedIndexers` autos become goal-driven: SHOOT goal held with the feed gate doing what `isReady()` did); fix the two probable path bugs (D9: Blue 2 Depot_Outpost start pose, Blue 3 to Depot global 1.0 m/s) **with the team in the loop**; delete orphans.
9. **Dashboard cleanup + docs** — §5 applied; docs/superstructure.md kept current; CLAUDE.md pruned of entries the rewrite made obsolete.

---

## 9. New folder structure (concrete)

```
src/main/java/frc/robot/
  Robot.java                     // Epilogue/logging bootstrap, MATCH_MODE, HootAutoReplay (kept)
  RobotContainer.java            // wiring only
  superstructure/
    Superstructure.java          // goal intake, manual mode, mechanism orchestration per state
    RobotState.java  Goal.java  Conditions.java  Transitions.java
    TurretAiming.java            // visual servo + gyro FF + dedupe (W3–W7)
  subsystems/
    drive/    Drive.java  CommandSwerveDrivetrain.java(frozen stock)  TunerConstants.java(frozen)  DriveConstants.java
    turret/   Turret.java  TurretIO.java  TurretIOReal.java  TurretIOSim.java  TurretWrap.java  TurretConstants.java
    hood/     Hood.java  HoodIO.java  HoodIOReal.java  HoodIOSim.java  HoodConstants.java
    shooter/  Shooter.java  ShooterIO.java  ShooterIOReal.java  ShooterIOSim.java  ShooterConstants.java
    intake/   Intake.java  IntakeIO.java  IntakeIOReal.java  IntakeIOSim.java  IntakeConstants.java
    indexer/  Indexer.java  IndexerIO.java  IndexerIOReal.java  IndexerIOSim.java  IndexerConstants.java
    vision/   Vision.java  VisionIO.java  VisionIOReal.java  VisionIOSim.java  VisionPoseEstimator.java  VisionConstants.java
  commands/   CalibrationCommand.java  (+ auto compositions if any outgrow inline factories)
  util/       ShotCalculator.java  TunableNumber.java
  RobotConstants.java            // loop period, controller ports, bus names, Gen flags
```

`Gen` feature flags and null-guarded construction survive (they saved the team this season) but the W29 braceless-guard class of bug dies with declarative binding guards.

---

## 10. Traceability table

Every FUNCTIONALITY_INVENTORY.md row → destination. "DELETED (Dn)" is a deliberate destination per the signed decisions. Zero unmapped rows.

### §1.1 Drivetrain
| Inventory row | Destination |
|---|---|
| Field-centric default drive (speeds/deadbands/inversion) | `Drive.teleopDriveCommand` — values verbatim in DriveConstants |
| X-stance brake (driver A) | `Drive.brakeCommand` |
| Seed field-centric (driver LB) | `Drive.seedFieldCentricCommand` |
| Idle while disabled | `Drive.idleWhileDisabledCommand` |
| Operator perspective periodic | frozen generated file (stock behavior) |
| Sim thread 4 ms | frozen generated file |
| SysId routines | stay in frozen generated file, unbound (D8 resolved in §4) |
| `getSwerveXSpeed/YSpeed` | `Drive.getRobotRelativeSpeeds()` |
| `configurePathPlanner()` | `Drive.configureAutoBuilder()` — PID/config verbatim |
| `@Logged` on drivetrain | removed from generated file; adapter logs instead |
| `addVisionMeasurement` overrides (dead) | get their caller: `VisionPoseEstimator → Drive.addVisionMeasurement` (D11) |
| TunerConstants (all values) | frozen verbatim, moved to subsystems/drive/ |

### §1.2 Turret
| Row | Destination |
|---|---|
| Boot config retry + zero + bus optimization + signal rates | `TurretIOReal` constructor (pattern verbatim) |
| `moveTurret` wrap+clamp returning commanded value | `Turret.setTargetPosition` + `TurretWrap.apply` (pure, tested) |
| Manual ±3 V | `Turret.setVoltage` (manual mode §7) |
| `atTurretSetpoint` (±0.005) | `Turret.atSetpoint` |
| Auto-aim 3-state command (MANUAL/PASSING/TRACKING) | dissolved into RobotState (MANUAL / PASS_* / IDLE+SPINNING_UP+SHOOTING) + `TurretAiming` helper |
| Gyro FF yaw-delta integration | `TurretAiming.applyGyroFF` via `Drive.getContinuousYawDeg` — sign/source verbatim (W3) |
| POV presets (0/−0.5/−0.25/0.25) | manual-mode + teleop bindings → `Turret.setTargetPosition` (values verbatim, TurretConstants) |
| RB re-zero | `Turret.zeroAtCurrentPosition()` bound onTrue (now a proper subsystem method — fixes the W37 requirement hole) |
| Interp tables + validity window | `ShotCalculator` verbatim |
| `isAimed()` (<3° last tx) | `Conditions.aimed` — plus fresh-frame-seen guard (W10 false-ready closed; flagged as deliberate behavior change in §11) |

### §1.3 Hood
| Row | Destination |
|---|---|
| Boot config + zero-at-bottom + rates | `HoodIOReal` (D4 confirms procedure) |
| `setHoodAngle` clamp [0, 0.08] | `Hood.setAngle` verbatim |
| Sag-follow default command | `Hood.holdCurrentPosition()` capture-once hold in IDLE (see §11 — behavior change, team-visible) |
| Presets X/Y/B | manual + teleop bindings, values verbatim |
| `atHoodSetpoint` | `Hood.atSetpoint` (still NOT in the feed gate — W13 preserved) |

### §1.4 Shooter
| Row | Destination |
|---|---|
| Boot config + rates | `ShooterIOReal` |
| `setShooterSpeed` upper clamp | `Shooter.setVelocity` — clamp both bounds (W36 edge closed, §11) |
| `atShooterSetpoint` ±1.67 | `Shooter.atSpeed` → `Conditions.atShooterSpeed` |

### §1.5 Intake
| Row | Destination |
|---|---|
| Pivot MotionMagic / roller velocity / NeutralOut stop | `IntakeIOReal` + `Intake` setpoints |
| `isOut()` = stow + 5° | `Intake.isOut` verbatim (W18) → `Conditions.intakeDeployed` |
| Over-travel recovery in periodic | `Intake.periodic` via nested machine (W16) |
| IntakeOutCommand deploy→latch→restow-on-end | nested machine STOWED→DEPLOYING→DEPLOYED_ROLLING→STOWING; A toggle → `Superstructure.toggleIntake` |
| IntakeInCommand | `Intake.requestStow()` (auto named `intakeIn` keeps working) |
| runRoller | DELETED (D8) |

### §1.6 Indexer
| Row | Destination |
|---|---|
| 3 velocity stages | `Indexer` + `IndexerIO` |
| indexerDefault (vertical 26.25 when intake out; missing requirements) | Superstructure IDLE/PASS_AIMING behavior: vertical 26.25 iff `intakeDeployed` — bug structurally impossible now (W20/D3) |
| LT manual vertical 35 | teleop + manual binding |
| reverseIndexers (−50% + shooter max forward) | UNJAMMING state behavior verbatim (W19, D15) |
| manualIndexer | DELETED (D8) |
| `feedIndexers` named command (35/35/100 ungated) | re-registered as Superstructure-level feed request (auto only) — same speeds |

### §1.7 Shooting commands
| Row | Destination |
|---|---|
| ShootCommand vision mode (tables/aim gate/passing branch/feed gating/end leaves hood) | SPINNING_UP/SHOOTING states + T5/T8; hood untouched on exit (W15) |
| ShootCommand manual ctor (fixed RPS, ignored `manual` param) | DELETED — unused in bindings; manual mode §7 covers fixed-RPS shooting (RT→55 RPS) |
| Shoot-while-moving dead math | `ShotCalculator.effectiveDistance`, toggle default OFF (D1) |
| AutoShootCommand | DELETED (D12); autos use SHOOT goal held (step 8) |
| CalibrationShootCommand (NT entries, turret pinned 0, ungated feed) | `CalibrationCommand` — workflow verbatim, binding stays commented |

### §1.8 Vision
| Row | Destination |
|---|---|
| Drain results, keep newest; sticky tracking; acquisition by lowest ambiguity | `Vision` targeting pipeline (W22 verbatim) |
| Hub vector math + −0.2 fudges + sign flip | `VisionConstants.TAG_TO_HUB_CENTER` verbatim (W24/W26, D6) |
| Passing normal-direction yaw | verbatim (W25) |
| Stale-distance hold | verbatim (W21) |
| Hub-before-passing precedence; alliance per-call | verbatim (W28) |
| No pose estimation | REPLACED: `VisionPoseEstimator` (D11) — targeting path unchanged |
| MAX_POSE_AMBIGUITY 1.0 | → 0.3 (D5, deliberate change) |
| FRC2026_WELDED.json unused | now loaded by `VisionPoseEstimator` (AprilTagFieldLayout) |

### §1.9 Dead/orphaned
| Row | Destination |
|---|---|
| LED + LEDConstants | DELETED (D8) |
| ClimbConstants + CANID CLIMB_MOTOR | DELETED, all references (D8) |
| manualIndexer, runRoller | DELETED (D8) |
| 5 orphan paths + empty folders | DELETED (D8) |
| SysId routines | frozen in generated file (§4) |
| CANID enum | replaced by per-subsystem constants as single source (IDs verbatim); name-lookup utility dropped (unreferenced) |

### §1.10 Robot lifecycle
| Row | Destination |
|---|---|
| Epilogue config + MATCH_MODE + DataLog/SignalLogger | Robot.java kept, importance per §5 |
| HootAutoReplay in robotPeriodic | kept verbatim (replay debugging is active practice, W33) |
| Auto schedule/cancel lifecycle | kept verbatim |

### §2 Weird Stuff W1–W40
| W | Destination (one line each) |
|---|---|
| W1 | `TurretWrap.apply` pure + TurretWrapTest |
| W2 | `Turret.setTargetPosition` returns commanded value; `TurretAiming` rebases accumulator |
| W3 | `TurretAiming.applyGyroFF` — delta integration, sign verbatim, via Drive adapter |
| W4 | omega sampled+logged in TurretAiming; compensation now lives (correctly) in ShotCalculator behind D1 toggle — the stub dies, its intent ships |
| W5 | dedupe verbatim in TurretAiming (shared-timestamp quirk documented; kept — changing it alters correction cadence) |
| W6 | visual servoing verbatim (target = current + tx/360); pose-based aiming still unused for the turret |
| W7 | manual rebase = manual-mode exit contract (§7) |
| W8 | boot-zero kept (D4); CANcoders stay telemetry; RB re-zero kept |
| W9 | config readback printout kept in TurretIOReal |
| W10 | `Conditions.aimed` + fresh-frame guard (deliberate fix, §11) |
| W11 | ShotCalculator.effectiveDistance, toggle OFF (D1) |
| W12 | T5/T8 + `shooterCommandedNonZero` + stale-distance speed hold (D2) |
| W13 | SHOOTING/PASSING entry+exit conditions; vertical always-on in shoot states; hood excluded from gate |
| W14 | PASS_SPINNING_UP/PASSING: hood 0.067 / 95 RPS verbatim; pass beats hub (passSelected checked first) |
| W15 | SHOOT-exit leaves hood at setpoint; only IDLE-entry-from-MANUAL recaptures (AutoShoot's hood-stop dies with it, D12) |
| W16 | Intake.periodic recovery, sim-reproduced (arm sim) |
| W17 | nested machine DEPLOYING→DEPLOYED_ROLLING latch + test |
| W18 | `Intake.isOut` verbatim threshold → `intakeDeployed` condition; idle vertical feed now lives in the INTAKING state |
| W19 | UNJAMMING behavior verbatim (D15) |
| W20 | structurally impossible (no default-command pattern for feed; Superstructure owns it) — D3 |
| W21 | Vision stale hold verbatim |
| W22 | sticky tracking verbatim |
| W23 | ambiguity gate 0.3 (D5) |
| W24 | −0.2 fudges verbatim (D6); flagged as input to the new turretToCamera calibration |
| W25 | passing normal yaw verbatim |
| W26 | sign flip verbatim |
| W27 | FIX-comment provenance → passing path covered by targeting parity checks in step 6 |
| W28 | precedence + caches verbatim; calculator cache cleared on enable + manual exit |
| W29 | dies with declarative guards in new RobotContainer |
| W30 | dies: suppliers injected, no public statics; aiming state lives in Superstructure |
| W31 | preserved by construction: drive default and turret aiming run during SHOOT states (Superstructure commands turret; drive untouched) |
| W32 | capture-once hold replaces sag-follow (§11 — team-visible change) |
| W33 | HootAutoReplay kept |
| W34 | MATCH_MODE kept |
| W35 | "Persian  Canivore" untouched (D10); bus names in RobotConstants with a warning comment |
| W36 | values preserved verbatim; lying comments corrected; negative-clamp edge closed (§11); kI=50/kP=100 hood gains kept as-is (tuned) |
| W37 | RB re-zero becomes `Turret.zeroAtCurrentPosition()` with proper requirement |
| W38 | step 8: leading-space path renamed when its auto is touched; D9 bugs fixed with team; settings.json dual-source documented in DriveConstants comment |
| W39 | stock template content — frozen as-is |
| W40 | dies: Superstructure null-safe wiring, no command-as-service |

### §3 Controls map
| Row | Destination |
|---|---|
| Driver sticks / A / LB | Drive adapter command factories (verbatim feel) |
| Driver B calibration (commented) | `CalibrationCommand`, binding stays commented |
| Operator A toggle intake | `Superstructure.toggleIntake` |
| Operator RT shoot | goal supplier → SHOOT |
| Operator LT vertical feed | direct binding (teleop + manual) |
| Operator RB turret re-zero | `Turret.zeroAtCurrentPosition` |
| Operator LB pass modifier | `Conditions.passSelected` / goal PASS |
| Operator left-stick-click + right X turret manual | absorbed into manual mode (§7) — Start toggles, right X jogs; single-axis-manual-without-mode is retired (§11) |
| Operator right-stick-click unjam | goal UNJAM |
| Operator POV turret presets | bindings → setTargetPosition (values verbatim) |
| Operator X/Y/B hood presets | bindings (values verbatim, guard fixed) |
| Default commands (drive/turret/hood/indexer) | drive: adapter factory; turret+hood+indexer defaults dissolve into Superstructure IDLE behavior |
| NEW: operator Start | manual mode toggle (§7) |

### §4 Autonomous
| Row | Destination |
|---|---|
| AutoBuilder config + chooser + lifecycle | `Drive.configureAutoBuilder` + RobotContainer (order preserved: configure → register → buildAutoChooser) |
| `intakeOut`/`intakeIn` | Superstructure intake requests |
| `shoot` | SHOOT goal held (passing false — verbatim semantics) |
| `stopAll` | `Superstructure.stopAll` |
| `autoShoot` + `feedIndexers` | redesigned in step 8: SHOOT goal with feed gate (replaces deleted AutoShootCommand, D12); same registered names so .auto files keep working until retuned |
| `reverseIndexer` | UNJAM goal request (null-guard bug dies) |
| 17 autos + 16 live paths + markers | preserved as-is on deploy; D9 fixes + orphan deletion in step 8 with team |

### §5 Dashboard — fully mapped in §5 above (every INPUT kept; every OUTPUT row dispositioned keep/strip).

### §6 Cross-subsystem findings
| Row | Destination |
|---|---|
| Pigeon yaw → turret FF | `Drive.getContinuousYawDeg` → TurretAiming |
| Speeds → aim logging / dead math | `Drive.getRobotRelativeSpeeds` → TurretAiming (log) + ShotCalculator (D1) |
| Vision → aim suppliers | Vision getters → Superstructure conditions |
| visionAutoAim-as-service | dies (W30) |
| intake.isOut → indexer default | `Conditions.intakeDeployed` |
| atShooterSetpoint → feed gate | T5/T8 |
| Motor getters for FaultMonitor | DELETED (D13) |
| Alliance flows (vision/calculator/perspective/PathPlanner) | each preserved in place |
| "Nothing influences drivetrain" | preserved; adapter is the future seam |

---

## 11. Deliberate behavior changes (the complete list)

Everything not listed here is behavior-preserving. Each of these is a signed decision or a bug fix with a regression test:
1. Lose-aim no longer commands 0 RPS; feed gate requires non-zero setpoint (D2, W12).
2. Vision ambiguity gate re-enabled at 0.3 (D5, W23).
3. Vision pose estimation added; odometry now vision-corrected (D11). Camera-on-turret handled with two measured constants + slew-reject gate (§3.6).
4. Shoot-while-moving implemented correctly, toggle default OFF (D1, W11).
5. `aimed` requires at least one fresh frame since enable (closes W10 false-ready window).
6. Shooter clamp closes the negative-input edge (W36; no current caller affected).
7. Hood idle hold is capture-once instead of sag-follow (W32) — **flag for driver feedback**: if the drooping hood was somehow load-bearing, revert is one line.
8. Turret single-axis manual (left-stick-click) is absorbed into full manual mode on Start (skill #5 requires full manual; keeping two overlapping manual systems violates "never fork near-duplicates"). **Operator must re-learn one button.**
9. AutoShootCommand deleted (D12); auto pre-spin expressed as SHOOT goal.
10. Deletions per D8/D13.
11. **CLEARING reverse back-out** (team-requested, Phase 2 review; amended at sign-off): releasing the shoot trigger mid-volley stops the shooter and reverses all indexers at full speed for kClearingSeconds (2.0 s), backing committed balls away from the flywheel instead of pinching them. The design's single atomic transition; SHOOT/UNJAM/MANUAL all exit or preempt it instantly.
12. **INTAKING is a visible RobotState** (team-requested, Phase 2 review): intake-only operation reads as INTAKING on the dashboard instead of IDLE+condition; intake-while-shooting concurrency unchanged (carried by the `intakeDeployed` condition through the shoot states).
