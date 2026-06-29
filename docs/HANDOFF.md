# Session Handoff — Rebuilt-2026 (skill-server + shift-aware autonomy)

Read this first if you're a fresh context picking up this work. It's the map of what exists, what
was done, the load-bearing gotchas, and what's open. Then read `CLAUDE.md` (codebase law + growth
rule), the architecture skill in `.claude/skills/frc-java-architecture/`, and the docs linked below
(`docs/strategy.md` is the current source of truth for the autonomy decision layer).

- **Repo:** `/Users/aaranchahal/Rebuilt-2026` — FRC team 2658, Java, WPILib 2026, CTRE Phoenix 6,
  command-based.
- **Branch:** `refactor/skill-server` (all work here; `main` is the PR target; `refactor/superstructure`
  is the PRE-migration spec — never modify it).
- **State:** `./gradlew build` green, Spotless-clean, **104 @Test methods / 0 failures**. Commit/push
  only when asked; end commit messages with
  `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- **Build offline:** `./gradlew build --offline` (env has no network; `timeout` cmd is absent on this
  mac — don't use it). JUnit sim tests need `HAL.initialize(500,0)` in `@BeforeAll` and
  `CommandScheduler.getInstance().unregisterAllSubsystems()` in setup+teardown.

## What this project is (the three layers)

1. **Skill-server runtime (done, earlier work).** Per-mechanism subsystems + IO triads and the
   `frc.robot.superstructure` package were replaced by a "skill-server" under `frc.robot.runtime`:
   actuators are DATA (`deploy/mechanisms.json` → generic `Mechanisms`), decisions are a declarative
   table (`deploy/skills.json` → stateless `SkillInterpreter` + the verbatim-ported `ScoringSequencer`
   reflex latch), reflexes are code (`runtime/reflex/*`), and the coprocessor seam is `SkillServer`
   (NT) + `LocalSkillDriver` (standalone). Drive is FROZEN CTRE behind `Drive`. Full detail:
   **`docs/skills.md`**.

2. **Shift-aware match autonomy (THIS session's main work).** A behavior tree (`frc.robot.autonomy`)
   that plays the whole **teleop** on its own, driven by an editable `deploy/strategy.json` + the 2026
   SHIFT clock. It decides WHEN/WHERE; it reuses the skills for HOW. Full detail: **`docs/strategy.md`**
   (read this — it supersedes the autonomy decision-layer parts of `docs/autonomy.md`, which now only
   documents the unchanged movement plumbing).

3. **Auto (20 s)** stays on the PathPlanner `.auto` routines from the chooser — the brain is teleop.

## What THIS session built (the shift-aware autonomy + sim tooling)

The big picture: the old collect↔shoot `MatchTree` became a **strategy-driven, shift-aware brain**
with dynamic obstacle avoidance and dashboard-driven sim controls. New/changed pieces:

- **`ShiftSchedule`** (pure, tested) — the 2026 teleop SHIFT clock: maps `getMatchTime()` → which
  25 s shift, and `getGameSpecificMessage()` + alliance → which HUB is active, into a `Mode`
  (`OUR_HUB_ACTIVE` / `OUR_HUB_INACTIVE` / `ENDGAME`). During alliance shifts only ONE hub scores;
  the strategy shoots when ours is active and harvests/repositions when it's not.
- **`StrategyConfig` + `deploy/strategy.json`** — THE editable strategy (named waypoints + per-mode
  plans). Same schema-validate-with-safe-fallback contract as skills.json/mechanisms.json. Edit this
  file (no recompile) to re-task the robot per match. `docs/strategy.md` is the guide.
- **`FieldGeometry`** — real 2026 field constants (hub centers reused from `VisionConstants`, alliance
  zone, keep-out rects for hubs/towers/trenches). Single source of truth for legality + the navgrid.
- **`LegalRegion`** (pure, tested) — clamps every nav target legal (never through hub/tower/trench,
  shoot only in our zone) AND `nearestClear(...)` diverts a target off a dynamic obstacle sitting on
  it.
- **`MatchTree`** (rewritten) — derives the mode each loop and dispatches: ACTIVE → shoot-from-a-REGION
  + roam-collect cycle; INACTIVE → harvest deep, then stage home `returnLeadSeconds` early; ENDGAME →
  score-cycle. Publishes `/Autonomy/{mode,cyclePhase,secondsUntilActive,modeOverrideActive}` to NT.
- **`MatchCycle`** — the collect↔shoot latch; SHOOT now holds until the feed gate has been open for
  `kShootEmptySeconds` (currently **3.0 s**) to empty the hopper by time (no ball sensor).
- **Dynamic obstacle avoidance** — `ObstacleProvider` seam → `PathPlannerNavigator` pushes
  `Pathfinding.setDynamicObstacles` every loop (boxes inflated by `kObstacleClearanceMeters`) and
  reschedules the in-flight route when obstacles change. `SimOpponentProvider` is a dashboard-driven
  fake opponent for proving it in sim; the real robot detector will just implement the seam.
- **`navgrid.json` regenerated** by `tools/gen_navgrid.py` from `FieldGeometry`, inflated by the real
  robot HALF-DIAGONAL (read from `settings.json`) at finer 0.2 m resolution — the committed navgrid was
  wrong (blocked the alliance zones) and under-inflated (clipped buildings).
- **Pose seed** — in sim the robot boots at the (0,0) corner (a blocked navgrid cell → can't path);
  `AutonomyCommand` seeds `AutonomyConstants.kSimStartPose` there.

## How to run it in sim (the EASY path — restart `simulateJava` after any code change)

`./gradlew simulateJava`, then in the dashboard (Glass/Elastic — repo ships `elastic-layout.json`):

1. **Auto chooser → "Shift Brain (autonomy)"**, then **Autonomous → Enable**. (One toggle; avoids the
   flaky teleop-trigger dance. The brain also runs in teleop when `TeleopAutonomy`=true, for real
   matches.)
2. **`AutonomyModeOverride`** (string) — `ACTIVE` / `INACTIVE` / `ENDGAME` / `MATCH`. This is how you
   flip "which hub is active" in sim without a practice match.
3. **`SimOpponentEnabled` / `SimOpponentX` / `SimOpponentY` / `SimOpponentOscillate`** — drop a fake
   opponent (renders on the same field via the `Pose` table) and watch the robot route around it.
4. Watch `/Autonomy/*` to see the brain's mode + cycle phase; `/skills/status/*` for scoring.

**Headless trace (a Claude session can read this itself):**
`./gradlew test --offline --tests "frc.robot.autonomy.AutonomySimObservationTest"` then
`cat build/autonomy-obs.txt` — a DETERMINISTIC (kinematic-navigator) sweep through every shift with a
per-shift feed-gate count. The real PathPlanner swerve sim is too jittery headless to demo shooting
(fires 0 one run, 34 the next), so the trace uses idealized movement; real pathfinding is exercised in
`simulateJava` + smoke-constructed in tests.

## Load-bearing gotchas (will waste your time if you miss them)
- **Restart `simulateJava` after ANY change.** `navgrid.json` and the dashboard controls are loaded at
  boot; a running sim won't have your edits (this bit us repeatedly this session).
- **PathPlanner pathfinds the robot as a POINT.** `settings.json` robot dims are only used by the
  PathPlanner GUI to inflate the navgrid; at runtime nothing inflates. We generate the navgrid
  ourselves (`tools/gen_navgrid.py`) and inflate by the half-diagonal; dynamic obstacles we inflate
  ourselves too (`kObstacleClearanceMeters`). Under-inflate → clips; over-inflate → seals the bump
  gaps to the neutral zone (keep them passable — the generator verifies this).
- **`setDynamicObstacles` NPEs if the pathfinder isn't initialized** — call `Pathfinding.ensureInitialized()`
  first (the navigator does). And the in-flight pathfind command won't replan on its own → the
  navigator reschedules when obstacles change. The planner DOES route around boxes (proved by
  `DynamicObstacleTest`); a target INSIDE a box is unreachable → `LegalRegion.nearestClear` diverts it.
- **Drive to a REGION, not a point.** Shoot fires from anywhere within `kShootRegionRadiusMeters`;
  collect ROAMS (`kCollectRoamRadiusMeters`). An opponent parked exactly on the nominal point used to
  make the robot drive through it (PathPlanner can't path into an obstacle).
- **`ShiftSchedule.kTeleopLengthSeconds = 140`** (10 + 4×25 + 30) drives every shift boundary off
  `getMatchTime()`. **`parseOurHubInactiveFirst` is a GUESS** (first char R/B) until the FRC Control
  System site publishes the game-data format — safe fallback = always-active. Fix only that method at
  kickoff.
- **Alliance flip:** the navigator uses `pathfindToPoseFlipped`; strategy poses are blue-authored and
  `LegalRegion` reasons in the blue frame. In sim, SET THE ALLIANCE deliberately.
- **Pose on the real robot** needs vision pose estimation ON (`VisionConstants.kTransformsMeasured`,
  currently false — measure the camera transform first). Sim pose is ground truth.
- Tuned numbers are sacred (gains, CAN ids, gear ratios, the −0.2 m hub fudge). CANivore is
  `"Persian  Canivore"` (double space). Mechanisms on bus `"mechanisms"`.

## Open items / what's next
- **Ball detection / possession** — the collect "roam" is a documented stand-in; real ball detection
  feeds the `PossessionProvider` + would replace roaming with go-to-detected-ball. Empty seam today.
- **Opponent detection** — `SimOpponentProvider` is the sim stand-in; the real detector implements
  `ObstacleProvider.dynamicObstacles()` and nothing else changes.
- **Field geometry is partly approximate** — hub/tower footprints are tag-anchored and solid; the
  TRENCH rectangles in `FieldGeometry` are APPROXIMATE pending real field CAD. Re-run
  `tools/gen_navgrid.py` after editing `FieldGeometry`.
- **`StalenessReflex` decision still unreconciled** (from the prior handoff): the migration added
  auto-degradation (stop shooter/feed when perception stale >1 s); the team's `problems.md` says
  auto-degradation was REMOVED by decision (they want manual takeover + fault REPORTING only). Not yet
  decided — surface it.
- **`CalibrationCommand`** was deleted at the skill-server cutover and not re-ported (shot-table
  recalibration has no in-code path).
- **`autonomy/DriveToNode.java` is now unused** by the rewritten `MatchTree` (left in place; delete if
  you want).
- Real-pathfinding sim is jittery headless — deterministic guarantees live in `MatchTreeTest` (fake
  navigator) + the pure tests; the GUI is the visual check.

## Where the new stuff lives
```
deploy/strategy.json                 the editable per-match strategy (waypoints + per-mode plans)
tools/gen_navgrid.py                 regenerates deploy/pathplanner/navgrid.json from FieldGeometry
src/main/java/frc/robot/autonomy/
  ShiftSchedule, StrategyConfig, FieldGeometry, LegalRegion   (new; pure, tested)
  MatchTree (rewritten), MatchCycle, AutonomyCommand, PathPlannerNavigator (changed)
  ObstacleProvider (wired), SimOpponentProvider (new sim opponent)
src/test/java/frc/robot/autonomy/
  ShiftScheduleTest, StrategyConfigTest, LegalRegionTest, SimOpponentProviderTest,
  DynamicObstacleTest (new), MatchTreeTest + AutonomySimObservationTest (rewritten)
docs/strategy.md (new, source of truth), docs/autonomy.md (movement plumbing only)
```

## Working norms with this user
- They value actually running/verifying (not guessing), honest diagnosis, and tests. They iterate fast
  in `simulateJava` and report what they see — reproduce their exact scenario headlessly before
  theorizing.
- Commit/push only when asked. They're fine pushing to `refactor/skill-server`.
- Persistent memory lives at the path in the system prompt's memory section + `MEMORY.md` index.
