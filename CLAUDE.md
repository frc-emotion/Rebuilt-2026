# Rebuilt-2026 Codebase Map

Architecture and style law lives in `.claude/skills/frc-java-architecture/`. Read that skill before touching code. This file is only the map of THIS codebase: facts and gotchas a developer needs that are specific to this robot.

## Growth rule for this file

An entry earns a place here only if BOTH are true:
1. It is specific to this codebase, not general Java/WPILib knowledge.
2. Getting it wrong wasted real time at least once, or would clearly have.

Entries are one or two lines each. When an entry turns out to be a general principle, promote it to the skill's reference files and remove it from here. Prune anything stale. Target length: under two pages, forever.

## Codebase facts

Seeded from FUNCTIONALITY_INVENTORY.md (Phase 1, 2026-06-09). W-numbers reference the inventory's Weird Stuff section.

- W1/W2: Turret wrap = setpoint ±1.0 rotation when past soft limits (−0.73/+0.39 rot), then clamp; the wrapped value is returned by `moveTurret` and fed back into the aim command's accumulator. Both halves are load-bearing.
- W3: Turret gyro FF integrates Pigeon2 yaw *deltas* (ADDED, not subtracted — sign encodes the turret/gyro convention). Rate-based version with 1.75 fudge was tried and abandoned (commented out).
- W4: `applyOmega()` is a deliberate stub; omega still sampled+logged for future retuning (`omegaFeedforwardMultiplier=0.05` unused).
- W5: Vision frame dedupe is exact-timestamp, shared between hub and passing reads — a fresh frame without the wanted tag burns the timestamp.
- W6: Turret aiming is pure visual servoing (target = current + tx/360); `TurretAimingCalculator.calculate()` (pose-based) is never called.
- W8: Turret AND hood are rotor-sensor zeroed at boot — robot must power on with turret straight forward and hood at bottom. Both CANcoders exist but are unfused (telemetry only). Operator RB re-zeroes turret mid-match.
- W10: `isAimed()` = last vision tx < 3°, init 0 → false-ready window before first frame.
- W11: Shoot-while-moving `effectiveDist` math computed but never used (lookups take raw `dist`).
- W12: Vision-mode ShootCommand commands 0 RPS when not aimed; tolerance-of-zero can re-open the indexer gate into a dead flywheel (fallback branch commented out).
- W13: Feed gating: vertical indexer always runs; horizontal+upward only when `atShooterSetpoint()` (±1.67 RPS), instantaneous re-close.
- W14: Passing shot is hardcoded hood 0.067 rot / 95 RPS and beats aimed; auto pins passing false.
- W15: ShootCommand.end() leaves hood holding; AutoShootCommand.end() stops hood. Deliberate.
- W16: Intake over-travel recovery lives in `Intake.periodic()` (re-commands stow if pushed past 0.14 rot while stowing).
- W17: IntakeOutCommand: rollers latch on at 15° tolerance; end() = auto-restow (toggleOnTrue A button).
- W18: `isOut()` = pivot >5° off stow (not "deployed") — vertical indexer runs at 26.25 RPS for the whole pivot transit.
- W19: reverseIndexers reverses all stages at 50% while spinning shooter FORWARD at clamp ceiling (400 RPS = flat-out) to eject pinched balls.
- W20: indexerDefault has NO addRequirements yet is installed as default command — should throw at init; unresolved.
- W21/W22/W23: Vision holds last-good distance on tag loss; sticky tag tracking prevents hub-face flip twitch; MAX_POSE_AMBIGUITY=1.0 (filter disabled; team decision: re-enable at 0.3 in refactor).
- W24: Every tag→hub vector carries a −0.2 m lateral fudge (hand-tuned aim calibration); X always −0.604, Z ignored.
- W25: Passing yaw = tag surface NORMAL direction (lob into the zone), not bearing-to-tag.
- W28: Tag set overlap means hub-before-passing classification order is load-bearing; calculator caches alliance until command restart.
- W29: Hood X/Y/B bindings: braceless if — Y and B escape the `hood != null` guard.
- W30: `RobotContainer.visionAutoAim` and `operator` are public statics; the aim command is also the robot-wide aiming-state service for shoot commands.
- W31: Teleop ShootCommand is deferred with requirements {indexer,hood,shooter} only — turret tracking and driving continue while firing.
- W32: Hood default = sag-follow (re-samples position each loop), not a true hold.
- W35: CANivore name is "Persian  Canivore" — DOUBLE SPACE is real. Mechanisms bus is "mechanisms".
- W36: Several constants comments lie (shooter peak 12 V not 10; TURRET_POS_RIGHT 0.25 rot = 90° not 18°; stale turret-limit comments). VALUES are law, comments are not. Shooter clamp is upper-bound only; PeakReverseVoltage=0.
- W38: PathPlanner: " Manual Depot to Outpost" has a leading space (referenced with it); Blue 2 Depot_Outpost starts mid-field with resetOdom (likely bug); Blue 3 to Depot is globally 1.0 m/s; 5 orphan paths; settings.json max speed 5.44 vs TunerConstants 5.85 (RobotConfig.fromGUISettings is a second source of truth).
- Vision never feeds drivetrain pose: `addVisionMeasurement` has zero callers; pose is odometry-only; field-layout JSON deployed but never loaded. Team decision: refactor MUST implement full pose estimation, robust with a single camera.
- Team-confirmed bugs in current code (fix in refactor, don't preserve): W12 (lose-aim 0 RPS ungates feed), W20 (indexerDefault requirements), shoot-while-moving (rebuild as toggle, default off). Recent partial rewrite means newer hand-written commands likely never ran on the robot.
- Deprecated/delete list (team, 2026-06-09): LED, all Climb references, runRoller, manualIndexer, 5 orphan paths, SysId routines, FaultMonitor getters, AutoShootCommand.
- Nothing in robot state influences the drivetrain (no heading snap / auto-aim drive); drivetrain yaw/omega flow INTO the turret.
- No game-piece sensing exists (no beam breaks/color sensors); `IntakeCurrentSpike=20` is a vestigial unused detection idea.
- Dead/orphaned: LED.java + LEDConstants (fully commented), ClimbConstants (no subsystem), runRoller (triply broken), manualIndexer (unbound), SysId routines (unbound), CANID enum (unreferenced registry duplicating per-file IDs).
- Telemetry: Epilogue @Logged everywhere + CTRE SignalLogger + stock Telemetry class; `MATCH_MODE=false` in Robot.java toggles NT verbosity; HootAutoReplay runs unconditionally in robotPeriodic.

## Learned gotchas

- Epilogue's annotation processor breaks on two `@Logged` classes with the same SIMPLE name in different packages (its generated binder single-type-imports both `FooLogger`s). That's why legacy classes had Epilogue stripped during the Phase 3 coexistence window.
- JUnit tests touching any WPILib sim class must call `HAL.initialize(500, 0)` in `@BeforeAll` — sim classes read battery voltage through the HAL and SIGSEGV natively without it.
