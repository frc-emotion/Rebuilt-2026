# Rebuilt-2026 Codebase Map

Architecture and style law lives in `.claude/skills/frc-java-architecture/`. Read that skill before touching code. This file is only the map of THIS codebase: facts and gotchas a developer needs that are specific to this robot.

## Growth rule for this file

An entry earns a place here only if BOTH are true:
1. It is specific to this codebase, not general Java/WPILib knowledge.
2. Getting it wrong wasted real time at least once, or would clearly have.

Entries are one or two lines each. When an entry turns out to be a general principle, promote it to the skill's reference files and remove it from here. Prune anything stale. Target length: under two pages, forever.

## Codebase facts

**Layout:** `superstructure/` owns the robot-level state machine (`Transitions` is pure and fully unit-tested — change it and `TransitionsTest` + `docs/superstructure.md` in the same PR). Mechanisms live in `subsystems/<name>/` with IO real+sim pairs. `frc.robot.legacy` is the pre-refactor archive: in the tree for reference, EXCLUDED from compilation via build.gradle sourceSets — never import it.

- **Frozen generated files:** `subsystems/drive/TunerConstants.java` and `CommandSwerveDrivetrain.java` are stock Phoenix Tuner X output — never edit (Spotless excludes them; verify byte-identical with git diff). Everything else talks to `Drive`, the thin adapter.
- **CANivore is named `"Persian  Canivore"` — the DOUBLE SPACE is the real device name.** Mechanisms are on a second bus, `"mechanisms"` (RobotConstants.kMechanismBus).
- **Boot procedure is load-bearing:** turret physically straight forward and hood at bottom hard stop at power-on. Both are rotor-zeroed in their IOReal constructors; their CANcoders are configured but unfused (telemetry only). Operator RB re-zeroes the turret mid-match.
- **Turret wrap** (`TurretWrap.apply`): setpoint past a soft limit (−0.73/+0.39 rot) gets ±1.0 full rotation then clamps; `Turret.setTargetPosition` returns the ACTUALLY-commanded value and `TurretAiming` must store it back or its accumulator winds up. Single ±1 correction only; decision ignores current position. All deliberate, all tested.
- **Turret gyro feedforward ADDS Pigeon yaw deltas** (`TurretAiming.applyGyroFf`) — the sign encodes the turret/gyro convention and is load-bearing. Aiming is pure visual servoing (target = current + tx/360); there is no pose-based turret aiming.
- **Vision frame dedupe is exact-timestamp and shared between hub and passing reads** — a fresh frame without the wanted tag burns the timestamp. Legacy quirk, kept deliberately.
- **Tag→hub vectors carry a hand-tuned −0.2 m lateral fudge on all 16 entries** (VisionConstants.TAG_TO_HUB_CENTER) — team-confirmed correct; do not "fix". Hub-before-passing tag classification order is load-bearing (the ID sets overlap).
- **Passing yaw aims along the tag's surface NORMAL** (lob into the zone the tag faces), not at the tag.
- **Pose estimation is hard-gated behind `VisionConstants.kTransformsMeasured` (currently false)** — inert until ROBOT_TO_TURRET / TURRET_TO_CAMERA hold real measurements. A guessed camera-on-turret transform silently poisons odometry; the gate is deliberate. Ambiguity gate is 0.3.
- **The feed gate is `aimed && atSpeed && shooterCommandedNonZero`** — the nonZero term prevents a 0-RPS setpoint from reading "at speed" and feeding a dead flywheel (the old robot's worst bug). Hood readiness is deliberately NOT in the gate (legacy behavior).
- **CLEARING is the single atomic transition:** shot release stops the shooter and reverses all indexers FULL speed for 2.0 s (`kClearingSeconds`) to back balls away from the flywheel. SHOOT re-press, UNJAM, and manual all exit/preempt it instantly; IDLE/PASS wait.
- **UNJAMMING spins the shooter FORWARD flat-out** (400 = clamp ceiling) while reversing indexers at half speed — intended, ejects pinched balls.
- **Intake has the robot's only nested state machine** (deploy latch at 15°, stow-stops-rollers-first, over-travel recovery below 0.14 rot while stowing). `isOut()` means >5° off stow, NOT "deployed" — the idle vertical feed (26.25 RPS) keys off it. Over-travel recovery is active from boot and will fight a manual pivot jog below the threshold — that's protection, not a bug.
- **Manual mode = operator Start.** Superstructure commands nothing; turret/hood jog via DEFAULT commands that no-op outside manual (a plain `manual.whileTrue(jog)` dies permanently the first time a preset interrupts it — whileTrue schedules on rising edge only). Manual PERSISTS across disable (deliberate: a sensor-dead robot stays manual); everything else resets to IDLE in `onEnable()`. POV turret presets and X/Y/B hood presets work ONLY in manual mode now.
- **Shoot-while-moving** lives in `ShotCalculator.effectiveDistance` behind NT toggle `Tuning/ShootWhileMovingEnabled`, force-set OFF at every boot.
- **Interp tables** (ShotCalculator) are calibrated field data (2026-03-17). Recalibrate via `CalibrationCommand` (binding commented in RobotContainer; requires manual mode active) → /Calibration NT entries.
- **Constants comments may lie; VALUES are law** (e.g. TURRET_POS_RIGHT 0.25 rot = 90°, not the commented 18°). Never round, convert, or "clean" a tuned number.
- PathPlanner loads robot config from `deploy/pathplanner/settings.json` (`RobotConfig.fromGUISettings`) — a second source of truth vs TunerConstants (5.44 vs 5.85 m/s max, known mismatch). The `.auto` files reference named commands by exact string: intakeOut, intakeIn, shoot, stopAll, autoShoot, feedIndexers, reverseIndexer. One path file has a LEADING SPACE in its name (" Manual Depot to Outpost") and is referenced with it — renaming breaks the auto.
- `MATCH_MODE` in Robot.java gates Epilogue NT verbosity (false = DEBUG+, true = CRITICAL only). HootAutoReplay runs unconditionally in robotPeriodic (deterministic log replay; no-op on a field).

## Learned gotchas

- Epilogue's annotation processor breaks on two `@Logged` classes with the same SIMPLE name in different packages (its generated binder single-type-imports both `FooLogger`s). That's why the legacy archive has no Epilogue annotations.
- JUnit tests touching any WPILib sim class must call `HAL.initialize(500, 0)` in `@BeforeAll` — sim classes read battery voltage through the HAL and SIGSEGV natively without it.
- Test classes that construct subsystems must `CommandScheduler.getInstance().unregisterAllSubsystems()` in setup AND teardown — SubsystemBase self-registers, and a leaked subsystem's periodic runs inside OTHER test classes' scheduler loops.
- `PhotonTrackedTarget`'s constructor asserts exactly 4 corners per corner list — synthetic test frames need real TargetCorner lists, not `List.of()`.
- Never re-add a second Phoenix6 vendordep json — duplicates share a UUID and Gradle's behavior is undefined (26.1.1 was deleted for this).
- The repo's working refactor docs (FUNCTIONALITY_INVENTORY/REFACTOR_DESIGN/REFACTOR_PROGRESS/docs/superstructure.md) are gitignored by team decision — they live on disk only. Keep docs/superstructure.md current anyway when transitions change (non-negotiable #8).
