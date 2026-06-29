# Shift-Aware Match Strategy

The robot plays the whole **teleop** autonomously, driven by an editable strategy file and the 2026
SHIFT clock. This doc is the source of truth for the strategy layer; update it when the strategy
schema, the shift model, the legality model, or the field geometry changes.

Auto (the first 20 s) is NOT here — it runs a PathPlanner `.auto` from the chooser. This layer is the
teleop brain.

## The 2026 SHIFT mechanic (why this exists)

Teleop (2:20 → 0:00, 140 s) is split into a 10 s **TRANSITION SHIFT**, four 25 s **ALLIANCE SHIFTS**,
and a 30 s **END GAME**. During the alliance shifts **only one alliance's HUB is active at a time** —
FUEL scored in an inactive hub is worth nothing, and you may only launch from inside your own
ALLIANCE ZONE (rule G407). Which hub is inactive in SHIFT 1 is set by who scored more FUEL in AUTO,
then it alternates. TRANSITION and END GAME have both hubs active.

So the optimal teleop is **shift-phased**: shoot only while our hub is live, and spend the off-shift
collecting/repositioning instead of feeding a dead hub.

## The three modes (`ShiftSchedule.Mode`)

`ShiftSchedule` (pure, `ShiftScheduleTest`) maps the teleop clock + the FMS auto-winner game data to:

| Mode | When | Behavior |
|---|---|---|
| `OUR_HUB_ACTIVE` | TRANSITION + our active shifts | Shoot + cycle on our side (the `MatchCycle` collect↔shoot latch). Each shot holds the feed open for `AutonomyConstants.kShootEmptySeconds` (7 s) to empty the hopper — there's no ball sensor, so we drain by time. |
| `OUR_HUB_INACTIVE` | the opponent's active shift | Harvest deep along the harvest route; within `returnLeadSeconds` of re-activation, stage home loaded. |
| `ENDGAME` | last 30 s (both hubs active) | Score-cycle the endgame plan (this robot has no climber). |

On the INACTIVE→ACTIVE handoff the cycle jumps straight to SHOOT (`MatchCycle.startShootPhase`) — we
staged home loaded, so we fire immediately instead of burning the collect dwell.

### Knowing which hub is active
`DriverStation.getMatchTime()` gives the shift (teleop counts down 140→0). `DriverStation
.getGameSpecificMessage()` carries the auto-winner the FMS relays at teleop start; `ShiftSchedule
.parseOurHubInactiveFirst` turns it (+ our alliance) into "is our hub inactive in SHIFT 1". **The wire
format of that message is published on the FRC Control System site, not the manual** — so the parser
is an isolated best-guess (first char `R`/`B` = inactive-first alliance) with a SAFE FALLBACK: an
unparseable/empty message → treated as **always active**, i.e. a plain legal score-and-cycle, never
harvesting through a shift it should have scored. Fix only that one method once the format is known.

## The editable strategy — `src/main/deploy/strategy.json`

The one file you change to re-task the robot per match / per alliance partner — no code change. Same
contract as `mechanisms.json`/`skills.json`: schema-validated at load with a safe fallback (invalid →
DriverStation error + a hardcoded legal score-and-cycle), loaded by `StrategyConfig`
(`StrategyConfigTest`).

- **`locations`** — named blue-origin waypoints `{x, y, headingDeg}`. The navigator alliance-flips
  them; you only ever author the blue side.
- **`plans`** — one per mode. `OUR_HUB_ACTIVE`/`ENDGAME` use `shootFrom` (a location) + `collectRoute`
  (a list of locations cycled one-per-collect). `OUR_HUB_INACTIVE` uses `harvestRoute` (cycled on
  arrival) + `stageAt`.
- **`returnLeadSeconds`** — how early, before our hub re-activates, harvest abandons collecting to
  stage home.

To change tactics ("cycle the left corner", "harvest the far depot", "stage closer to the hub") edit a
route array or a coordinate and re-run the sim — no recompile.

## Legality is enforced, not assumed

Every nav target is run through `LegalRegion` (pure, `LegalRegionTest`) before the navigator sees it:
clamped on-field, pushed out of every `FieldGeometry` keep-out (both hubs, both towers, the
too-tall-to-pass trenches), and — for shoot poses — forced inside our own alliance zone (G407). You
**cannot** author a coordinate that drives through the hub, into the tower base (the implicit "no
climb" boundary), or out of our zone; the worst a bad coordinate does is snap to the nearest legal
point. `FieldGeometry` is the single source of truth for all of it, reusing the tuned hub centers from
`VisionConstants`.

### The navgrid mirrors FieldGeometry
`deploy/pathplanner/navgrid.json` (the pathfinder's static obstacle map) is generated from the same
keep-outs by `tools/gen_navgrid.py` — run it after changing `FieldGeometry`. It keeps the alliance
zones open (the old committed grid wrongly blocked most of the blue zone, including the shoot/collect
poses) while blocking the hubs/towers/trenches, inflated by the robot clearance. `LegalRegion` is the
hard guarantee; the navgrid is the planner's hint — they share one source so they cannot drift.

## Running it

- **Enable on the robot:** the brain takes over teleop ONLY when the dashboard boolean
  `TeleopAutonomy` is true (default false), so manual driving is always available (non-negotiable #5).
  Auto stays on the PathPlanner routine.
- **Deterministic tests:** `MatchTreeTest` (fake navigator) asserts active-shift collect→shoot+fire,
  inactive-shift harvest-into-neutral-without-firing, and the re-activation SHOOT jump.
- **Headless sim sweep:** `AutonomySimObservationTest` runs the real PathPlanner navigator + swerve
  sim with a compressed teleop clock through every shift and dumps `build/autonomy-obs.txt` — read it
  to watch mode/shift/pose/phase/skill per loop. Observation-only (the real swerve sim is
  timing/thread dependent; the guarantees live in `MatchTreeTest`).

## Open seams (unchanged by this layer — only the providers fill in)
- `PossessionProvider` — ball count / "full" (no sensor; harvest uses the dwell/route instead).
- `ObstacleProvider` — opponents + the tower base for dynamic keep-outs (no opponent perception yet).
- `NavCostProvider` — a learned local-nav cost term (no inputs yet).
- Human-takeover — `() -> false` (full autonomy is the goal).
- Pose on the real robot needs `VisionConstants.kTransformsMeasured` flipped on (sim pose is ground
  truth). The strategy file is also the seam for the future "just name an area" once ball-detection +
  robot-avoidance land.
