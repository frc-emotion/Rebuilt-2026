# REFACTOR_PROGRESS.md — 2026 Refactor

Kept current for the duration of the refactor. One source of truth for where we are.

## Current phase

**Phase 3 — Rewrite, on branch `refactor/superstructure`.** All rewrite-order items implemented; final soft gates running. Robot still boots LEGACY code (`frc.robot.Main` → `frc.robot.legacy.Robot`) until Phase 4 cutover.

## Phase 3 log

| Gate | Status | Notes |
|---|---|---|
| Setup: package move to `frc.robot.legacy` | DONE | Mechanical; build green. Generated files verified via git diff: only package/import lines changed. Legacy Epilogue annotations stripped (processor breaks on duplicate simple names — see CLAUDE.md gotchas); legacy keeps DataLog/SignalLogger/Telemetry NT. |
| Skeleton: RobotConstants, TunableNumber, ShotCalculator | DONE | Tables verbatim; shoot-while-moving rebuilt behind `Tuning/ShootWhileMovingEnabled` NT toggle, default false, force-off at boot (D1). Legacy calculator's dead `calculate(Pose2d)`/alliance-cache path dropped (was uncalled in legacy). |
| Shooter (pattern proof) | DONE — gate PASS | Zero value differences. Deliberate: clamp closes negative edge (§11.6). Gate caution honored: feed gate uses `commandedNonZero`. 3 sim tests. |
| Hood + Indexer | DONE — gate PASS | Zero value differences. Capture-once hold per §11.7. Watch item: `kManualVolts=2.0` and `kManualPivotVolts=2.0` are NEW unsigned numbers (manual-mode jog scales) — need team blessing at tuning. Sim-gain duplication nit fixed post-gate. |
| Intake + Turret | DONE — gate PASS (11/11 tests) | Wrap semantics verbatim + tested; nested machine reproduces W16/W17/W18. Gate findings addressed: added `isDeployed()`; removed duplicate Phoenix6-26.1.1 vendordep (shared UUID with 26.3.0). Acknowledged divergences: over-travel recovery now live at boot (strictly more protective); recovery can override manual pivot jog (protection wins). |
| Drive adapter | DONE | Generated CommandSwerveDrivetrain restored to STOCK in `subsystems/drive` (hand-additions → Drive adapter; diff shows exactly: package/imports, getSwerveX/YSpeed, configurePathPlanner, stale Epilogue javadoc). TunerConstants byte-identical except package/import. Both now FROZEN per drivetrain policy. |
| Vision + pose estimation | DONE — gate running | Targeting pipeline ported verbatim (incl. FIX semantics, sticky tracking, −0.2 fudges, normal-direction passing yaw). MAX_POSE_AMBIGUITY = 0.3 (D5). NEW VisionPoseEstimator: ~120 lines, photonlib-documented pattern, three gates (turret-slew, ambiguity, on-field), two-tier std devs. **Pose estimation is HARD-DISABLED via `VisionConstants.kTransformsMeasured=false` until the team measures ROBOT_TO_TURRET and TURRET_TO_CAMERA** — placeholders would poison odometry. |
| Superstructure + Transitions + tests | DONE — gate running | T0–T24 implemented as exhaustive arrow switches; CLEARING is the single atomic transition (2.0 s full-reverse back-out, team-amended). TurretAiming ports W3–W7 verbatim + W10 fresh-frame fix. 44 transition tests. |
| Manual mode | DONE | Operator Start toggles; turret/hood jog via no-op-outside-manual default commands; RT=55 RPS, LT=vertical 35, right-stick=unjam mix, POV/X-Y-B presets manual-gated; exit re-syncs to IDLE. |
| RobotContainer + Robot + bindings + autos | DONE — gate running | Named commands registered with EXACT legacy strings; `autoShoot`/`feedIndexers` → held SHOOT goal (D12 redesign); nothing in src/main/deploy touched. Full-robot sim smoke test passes (constructs, runs, manual round-trip, INTAKING via arm sim, SHOOT-no-vision holds safely). |

**Tests: 65 green** (`./gradlew build` + test). Suites: TransitionsTest 44, TurretWrapTest 6, IntakeSequencingTest 5, ShooterSimTest 3, HoodIndexerSimTest 2, RobotContainerSmokeTest 5.

## Implementation decisions made during Phase 3 (beyond the signed design)

1. Legacy lost Epilogue telemetry during the coexistence window (annotation-processor name collision). Returns at cutover.
2. POV turret presets and X/Y/B hood presets are **manual-mode-only** in the new robot (design §7 lists them under manual; outside manual the Superstructure owns those mechanisms). LT vertical feed still works in normal mode (flows through the Superstructure as a condition).
3. Legacy ShotCalculator's never-called pose-based `calculate()` + alliance cache dropped (dead code; the live alliance behavior is Vision's per-call check, preserved).
4. Over-travel recovery active from boot (legacy was inert until first stow command) — strictly more protective.
5. `reverseIndexer` named command no longer NPEs when shooter is disabled (legacy W—k bug class dies with declarative guards).

## Hardware/team TODOs before Phase 4 cutover

- **Measure ROBOT_TO_TURRET and TURRET_TO_CAMERA** (tape measure, 3 numbers each + angles), set them in `subsystems/vision/VisionConstants.java`, flip `kTransformsMeasured = true`. Pose estimation is inert until then.
- Bless the two new manual-jog scales: `HoodConstants.kManualVolts = 2.0`, `IntakeConstants.kManualPivotVolts = 2.0`.
- Tune `kClearingSeconds` (2.0 s) and `kFollowThrough`-replacement behavior on carpet.
- Spotless + CI pipeline not yet configured in this repo (non-negotiable #9) — add at/with cutover PR.
- Operator briefing: Start = manual mode; left-stick-click turret manual is retired; POV/X-Y-B presets now live inside manual mode.

## Next step

- Final two soft gates (vision+drive, superstructure+integration) — fix anything they find.
- Then STOP. **Phase 4 (cutover) is a separate session:** point Main at frc.robot.Robot, delete frc.robot.legacy, delete orphan paths/autos cleanup (D8 deploy items were deferred to keep deploy untouched in Phase 3), final on-robot verification.
