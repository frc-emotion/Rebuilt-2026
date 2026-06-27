# v1 Match Autonomy (behavior tree)

A behavior tree that plays the whole match on its own with what exists **today** — the skills from
the skill-server migration + PathPlanner runtime pathfinding. It decides **when** and **where**, and
reuses the existing skills for **how**; it does not re-implement shooting/intaking. Lives in
`frc.robot.autonomy`.

Select it from the dashboard auto chooser as **"BT Match (autonomy)"**. In `simulateJava` it drives
collect↔shoot cycles with zero human input. Logic is locked by `MatchTreeTest` (≥2 autonomous cycles
against the real skill runtime).

## How it moves — PathPlanner `pathfindToPose`, not pre-drawn paths

Movement goes through the `Navigator` seam. The production `PathPlannerNavigator` calls
`Drive.driveToPose(target)` → `AutoBuilder.pathfindToPoseFlipped(target, constraints)`: **runtime,
on-the-fly pathfinding over the deployed `navgrid.json`**. Each decision ("go to the shoot pose",
"go to the collection region") generates a **fresh route from the current pose** — many small dynamic
paths, one per decision, **not one big path and not the pre-drawn `.path`/`.auto` files** (those stay
for the static auto chooser). Targets are blue-origin; `pathfindToPoseFlipped` mirrors them for red.

- Static-field obstacle avoidance: free, from the navgrid.
- **Dynamic** (opponent) avoidance: NOT available — no opponent perception yet (see Seams).
- Pose source is `Drive.getPose()`. In sim that's ground truth; on the real robot this needs the
  vision pose-estimation gate flipped on (`VisionConstants.kTransformsMeasured`, currently false).

## The tree

Root **Fallback** (priority selector), ticked every loop:

1. **Human-takeover gate** (stub) — if a takeover signal is set, stop and idle. Wired to `() -> false`
   today; this is also why the tree runs as the autonomous command, so it yields cleanly to the
   driver.
2. **Endgame branch** — in the last `kEndgameSeconds`, drive to a park pose and hold. This robot has
   **no climb** (out of scope), so it's a documented park stub. Inert in sim (match time < 0).
3. **Collect ↔ Shoot cycle** — a Fallback:
   - **Shoot** (`Sequence`): only when the cycle latch says shoot → drive to a safe shoot pose
     (idle while traveling) → hold the `shoot` skill. We can score from anywhere on our side, so this
     is "park and shoot" — **no aim-on-the-move**; the turret auto-aims via the skill when parked.
   - **Collect** (`Sequence`): drive to the collection region with the intake out → dwell and sweep.

```
Fallback (root)
├── Sequence  humanTakeover?  → stop + idle
├── Sequence  isEndgame?      → driveTo(park) → hold            [park stub, no climb]
└── Fallback  (cycle)
    ├── Sequence  inShootPhase? → [idle while travel → driveTo(shootPose)] → shoot
    └── Sequence              [idle + intakeOut while travel → driveTo(sweep)] → idle + intakeOut
```

### Anti-thrash: the cycle latch (`MatchCycle`)
A pure stateless tree would flip branches the instant a condition wobbled. Instead the "should we
shoot?" signal is debounced in `MatchCycle` with minimum dwells: **COLLECT** is held ≥
`kCollectSeconds` (or until positively full), **SHOOT** ≥ `kMinShootSeconds` and ≤
`kShootTimeoutSeconds`. "Full" comes from `PossessionProvider` (always false — no sensor), so the
collect dwell timer is what triggers shooting; "shot confirmed" is the scoring status reaching
`SUCCEEDED` (feed gate open), used as an early-exit, not required.

### Where this deviates from the literal spec (and why)
- **"full OR dwell-expired" → dwell timer only.** No ball sensor exists, so "full" is unreadable;
  `PossessionProvider` is the seam, the dwell is the trigger.
- **The shoot node uses the skill STATUS contract**, not a blind shoot timer — it holds `shoot` and
  the cycle leaves the shoot phase on `SUCCEEDED` (early) or the timeout (cap). This is what the
  status/timeout instrumentation was built for.
- **Movement is abstracted behind `Navigator`** rather than wiring PathPlanner straight into the
  tree — it makes the decision layer unit-testable with a fake and gives the obstacle/cost seams a
  clean home.

## Empty seams (documented, no-op today)
- `ObstacleProvider` — dynamic obstacles (opponents) for the navigator. Empty: no opponent
  perception (single turret AprilTag camera). When detection lands, feed positions here.
- `NavCostProvider` — a learned local-nav cost term over the pathfinder. **No MLP** — it has no
  inputs yet; `ZERO` adds no cost. The hook is where a model's per-point cost would blend into the
  planner.
- `PossessionProvider` — ball count / "full". `UNKNOWN` today (no beam-break).
- Human-takeover signal — `() -> false` today.

These are the future-work seams from the team's `problems.md` (#4 opponents, #6 possession, the
learned-nav term). The tree does not change when they're filled — only the providers do.

## Tuning (`AutonomyConstants`)
All field poses are **placeholders** in blue-origin meters (2027 geometry unknown), seeded from real
deploy-path anchors so the sim drives between two distinct points. Dwell/timeout/sweep timings and
the pathfinding constraints (clamped to the PathPlanner 5.44 m/s, not TunerConstants' 5.85) live
here too. When the real field is known, only this file changes.

## Files
- `bt/` — the tiny framework: `Status`, `Node`, `Fallback`, `Sequence`, `Leaf`.
- `Navigator` + `PathPlannerNavigator` + `DriveToNode` — movement.
- `MatchCycle` — the dwell latch.
- `MatchTree` — the tree assembly.
- `AutonomyCommand` — schedules the tree as the autonomous routine.
- `AutonomyConstants` — field poses + timings.
- seams: `ObstacleProvider`, `NavCostProvider`, `PossessionProvider`.
- `Drive.driveToPose(...)` — the one adapter method added (generated drive files untouched).
