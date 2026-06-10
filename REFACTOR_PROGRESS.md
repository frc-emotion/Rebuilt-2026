# REFACTOR_PROGRESS.md — 2026 Refactor

Kept current for the duration of the refactor. One source of truth for where we are.

## Current phase

**Phase 2 — Design (no robot code changes).** FUNCTIONALITY_INVENTORY.md signed off. REFACTOR_DESIGN.md and docs/superstructure.md produced. **Awaiting sign-off on REFACTOR_DESIGN.md before any robot code is written.**

## Done

- Skill amended: vendor-generated code exemption (non-negotiable #2), CTRE sim instead of maple-sim default, new "Drivetrain policy" section in `references/architecture.md`.
- Full codebase ingested via 8 parallel read-only subagents (subsystems, drivetrain, RobotContainer/Robot, constants, PathPlanner deploy tree).
- `FUNCTIONALITY_INVENTORY.md` created: per-subsystem behavior tables, 40-entry Weird Stuff section with quoted code, full controls map, autonomous inventory, dashboard I/O classification, cross-subsystem map.
- `CLAUDE.md` "Codebase facts" seeded from Weird Stuff.
- Key structural findings: vision is turret-targeting only (drivetrain pose = pure odometry, `addVisionMeasurement` has zero callers); nothing influences the drivetrain from robot state; `visionAutoAim` (public static command) doubles as the robot-wide aiming state service; LED is fully commented out; Climb is orphaned constants with no subsystem; no game-piece sensing exists anywhere.

## Checklist answered (2026-06-09)

All 15 Phase 1 confirmation questions answered by the team; decisions recorded in FUNCTIONALITY_INVENTORY.md § "Phase 1 verification decisions". Highlights:
- **New requirement:** complete vision pose estimation feeding drivetrain odometry, robust with a single camera. Ambiguity gate returns at 0.3.
- **Confirmed bugs to fix in refactor:** lose-aim→0 RPS indexer gate (W12), indexerDefault requirements (W20), shoot-while-moving (rebuild as toggle, default OFF).
- **Deletions:** LED, all Climb references, runRoller, manualIndexer, 5 orphan paths, SysId routines, FaultMonitor leftovers, AutoShootCommand (deprecated — auto `autoShoot`/`feedIndexers` pattern must be redesigned).
- **Confirmed sacred:** −0.2 m tag fudge, boot-zeroing procedure, "Persian  Canivore" double space, TURRET_POS_RIGHT=0.25, reverseIndexers shooter-forward unjam, hood end-behavior asymmetry.
- Recent partial rewrite means several commands likely never ran on the robot — treat hand-written newer commands with suspicion.

## Phase 2 done (2026-06-09)

- REFACTOR_DESIGN.md: RobotState (10 states) + Goal (5 goals) derived from implicit old-code states; full transition table T0–T24 with exactly ONE atomic transition (FINISHING follow-through drain — team-requested); per-mechanism contracts (Intake gets the only nested machine, Turret deliberately does NOT); Drive adapter API (generated file restored to stock and frozen); pose estimation kept maximally simple (two measured Transform3d constants — placeholders until team measures — one compose line, three gates, photonlib-documented pattern verified via Context7); dashboard keep/strip plan; manual mode on operator Start with IDLE re-sync; named test plan; 9-step rewrite order; full traceability table (§10, zero unmapped rows); deliberate-behavior-changes list (§11, 12 items).
- Phase 2 review revisions (team): INTAKING added as a visible RobotState (concurrency preserved via intakeDeployed condition); FINISHING atomic drain added (≤0.30 s, UNJAM/MANUAL still preempt); pose estimation simplified per "everything in the current codebase is overcomplicated" directive.
- docs/superstructure.md: Mermaid diagrams (scoring chain + intake nested machine).
- Two-direction controls verification was deferred by the team; inventory treated as signed off per Phase 2 instruction.

## Next step

- **BLOCKED ON: team sign-off of REFACTOR_DESIGN.md.** No robot code until then. Pushback resolved by revising the design, not deferring to the rewrite.
- Then Phase 3 in the §8 order (skeleton+deletions → simple mechanisms → intake → turret → drive adapter → vision → superstructure → autos → dashboard).
- Tooling: Context7 MCP verified and now natively connected — use for all WPILib/Phoenix/PathPlanner/PhotonVision doc lookups (also mandated in the skill).

## Open questions

- PathPlanner probable bugs (team unsure): Blue 2 Depot_Outpost mid-field resetOdom; Blue 3 to Depot global 1.0 m/s. Resolve in auto phase.
- How autos replace the deprecated AutoShootCommand pattern (8 autos use `autoShoot`/`feedIndexers`).
- Camera count/placement for the new pose estimation (currently one turret-mounted camera; "may be limited to one camera at times" — is a second camera planned?).
