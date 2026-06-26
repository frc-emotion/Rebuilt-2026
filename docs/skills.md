# Skill-Server Architecture

Source of truth for the robot-level decision system after the skill-server migration. This file
supersedes the old `superstructure.md`. Any PR that changes a skill, a reflex, a setpoint source, or
the phase transitions MUST update this file (non-negotiable #8).

The old `frc.robot.superstructure` package (Goal / RobotState / Transitions / Superstructure) and the
six per-mechanism IO triads are **deleted**. They are replaced by four things sorted by *what each
thing is*:

1. **Actuator hardware as DATA** — `src/main/deploy/mechanisms.json` + the generic mechanism layer
   (`frc.robot.runtime.Mechanisms`).
2. **A declarative skill table** — `src/main/deploy/skills.json` + the stateless interpreter
   (`frc.robot.runtime.SkillInterpreter`).
3. **A fixed, tested reflex library** — `frc.robot.runtime.reflex.*` (real-time, stays code).
4. **Computed setpoints as a named-function registry** — `frc.robot.runtime.Setpoints` (interp
   tables) + `frc.robot.runtime.TurretAiming` (aim geometry).

Everything is behavior-preserving: the `refactor/superstructure` branch is the specification, and the
Phase-6 setpoint-equivalence harness proved per-tick identical mechanism commands across the W# grid
before cutover. The permanent behavior catalog is now `SkillInterpreterFlowTest` (flows) +
`ScoringSequencerTest` (the pure transition logic).

## Concurrent axes

The interpreter runs a small fixed set of axes every loop:

- **intake** — orthogonal to scoring, runs in EVERY mode (including manual). The intake nested
  machine (`IntakeReflex`) deploys/stows and exposes `isOut()`.
- **scoring** — idle / shoot / pass / unjam, driven by the scoring-phase sequencer.
- **drive** — untouched: the CTRE `Drive` adapter + PathPlanner, separate from the runtime.

## Mechanism layer (`mechanisms.json`)

One entry per non-drive mechanism motor: `shooter`, `turret`, `hood`, `indexerVertical`,
`indexerHorizontal`, `indexerUpward`, `intakePivot`, `intakeRoller`. Each entry is the tuned
TalonFX config as data — CAN id + bus (`mechanisms`), control type, gear ratio (kept as an exact
`a/b` fraction string so it reproduces the legacy Java double bit-for-bit), Slot0 gains, MotionMagic,
current limits, inversion, neutral mode, soft limits, feedback source (rotor / fused / remote
CANcoder), an optional output clamp (the legacy subsystem clamps: shooter `[0,400]` RPS, hood
`[0,0.08]` rot), boot-zero, and the sim plant + emulated controller. Boot-zeroing is preserved:
turret + hood rotor-zero at construction (turret straight forward, hood at the bottom hard stop); the
intake uses its absolute fused CANcoder. `Mechanisms` exposes `set*`/`read`/`commandedRps`/
`commandedPositionRot` per name. **Schema-validated with a safe fallback**: a missing/malformed file
loads no mechanisms (idle, disabled-safe) and reports a DriverStation error.

## Skill table (`skills.json`)

Invokable scoring skills (the operator/coprocessor request these by name):

| Skill | turret source | hood | shooter | feed | reflexes |
|---|---|---|---|---|---|
| `idle` | `turret.aimHub` | hold (capture-once) | off | rest | turretTracking, staleness |
| `shoot` | `turret.aimHub` | `hood.shotCalc` | `shooter.shotCalc` | gatedHub | feedGate, clearing, staleness |
| `passAim` | `turret.aimPassing` | uncommanded | off | rest | turretTracking |
| `passShoot` | `turret.aimPassing` | `hood.passing` | `shooter.passing` | gatedPass | passGate, clearing |
| `unjam` | `turret.aimHub` | uncommanded | `shooter.unjamMax` | unjam | turretTracking |
| `intake` (axis) | — | — | — | — | the intake reflex |

The JSON references named functions/predicates/reflexes from Java registries — it never embeds
arithmetic or control flow. `aimHub`/`aimPassing` are named setpoint sources in `TurretAiming`;
`shotCalc`/`passing`/`unjamMax` are named scalar sources in `Setpoints`. Same safe-fallback contract
as the mechanism layer (an invalid table forces a hardcoded safe-idle).

## Scoring-phase sequencer (the tested transition logic)

The interpreter is stateless except for enumerated reflex latches. The one that gives the scoring
axis its one-loop-per-transition timing (which a purely stateless gate cannot reproduce) is the
`ScoringSequencer` — the legacy `Transitions` logic, ported verbatim (T# IDs preserved). The
high-level skill maps to a goal + conditions; the sequencer advances one phase per loop; the
interpreter executes that phase's mechanism setpoints.

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> INTAKING : T0 intake toggled on
    INTAKING --> IDLE : T0 intake toggled off

    IDLE --> SPINNING_UP : T1 SHOOT, !pass
    IDLE --> PASS_SPINNING_UP : T2 SHOOT, pass
    IDLE --> PASS_AIMING : T3 PASS
    IDLE --> UNJAMMING : T4 UNJAM

    SPINNING_UP --> SHOOTING : T5 aimed && atSpeed && shooterCommandedNonZero (W12)
    SPINNING_UP --> PASS_SPINNING_UP : T6 pass
    SPINNING_UP --> IDLE : T7 release (no balls committed, no clear)

    SHOOTING --> SPINNING_UP : T8 gate lost (W13/W12)
    SHOOTING --> PASS_SPINNING_UP : T9 pass
    SHOOTING --> CLEARING : T10 release [ATOMIC]
    SHOOTING --> UNJAMMING : T10u UNJAM

    PASS_AIMING --> PASS_SPINNING_UP : T11 SHOOT
    PASS_AIMING --> IDLE : T12 release
    PASS_AIMING --> UNJAMMING : T13 UNJAM

    PASS_SPINNING_UP --> PASSING : T14 atSpeed only (W14)
    PASS_SPINNING_UP --> SPINNING_UP : T15 !pass
    PASS_SPINNING_UP --> PASS_AIMING : T16 release, LB held

    PASSING --> PASS_SPINNING_UP : T17 !atSpeed
    PASSING --> SPINNING_UP : T15m LB released while firing
    PASSING --> CLEARING : T18 release [ATOMIC]
    PASSING --> UNJAMMING : T18u UNJAM

    CLEARING --> IDLE : T22 clearingElapsed (to REST)
    CLEARING --> SPINNING_UP : T23 SHOOT re-requested
    CLEARING --> UNJAMMING : T24 UNJAM preempts the clear

    UNJAMMING --> IDLE : T19 release (to REST)

    state MANUAL
    note right of MANUAL
        T20: reachable from EVERY phase via operator Start (never blockable, even
        mid-CLEARING). The interpreter commands nothing in manual except the intake
        axis; the local driver drives mechanisms directly. PERSISTS across disable.
        T21: exit re-syncs to IDLE (aiming reset, never the pre-manual phase).
    end note
```

`REST = intakeDeployed ? INTAKING : IDLE`. The single ATOMIC transition is `SHOOTING/PASSING →
CLEARING`: on shot release the shooter stops and all indexers reverse FULL for `kClearingSeconds`
(2.0 s) before an IDLE/PASS request is honored; SHOOT re-press / UNJAM / MANUAL exit instantly.

## Reflex library (real-time floor — stays code, never data, never moves to a coprocessor)

- **Feed gate (W12/W13)** — hub firing opens only `aimed && atSpeed && shooterCommandedNonZero`;
  the nonZero term stops a 0-RPS setpoint from feeding a dead flywheel. Hood readiness deliberately
  excluded. Encoded in the sequencer's T5.
- **Pass gate (W14)** — passing fires on speed ONLY. Sequencer T14.
- **Clearing back-off** — the atomic full-reverse drain (timer + phase), `ClearingReflex` folded into
  the sequencer + interpreter timer.
- **Intake nested machine (W16/W17/W18)** — deploy latch at 15°, stow-stops-rollers-first, over-travel
  recovery below 0.14 rot. `IntakeReflex`.
- **Turret tracking / gyro feedforward (W1–W5, W10)** — pure visual servo + ADDITIVE gyro yaw-delta
  feedforward (sign is load-bearing) + the ±1 wrap with single correction + clamp + exact-timestamp
  vision dedupe. `TurretAiming` + `TurretWrap`.
- **Hood capture-once (W32)** — `HoodHold`.
- **Staleness → safe (NEW)** — if the perception pipeline delivers no fresh frame for
  `kPerceptionStaleSeconds` (1.0 s), a firing skill backs the shooter + feed off. Inert in normal
  operation; the one behavior added by the migration.
- **Disabled-output lockout / brownout / motor-safety** — `SafetyReflex.lockout` neutralizes every
  mechanism while disabled; brownout tolerance is the per-mechanism current limits (now data in
  `mechanisms.json`, applied to every Talon); closed-loop velocity/position runs on the Talons.

## NetworkTables skill API (`SkillServer`)

The seam a future coprocessor drives the robot over. Pure transport — it never decides.

- **`/skills/request/`** (inbound): `skill` (string), `manualMode`, `manualFeed`, `intakeDeploy`
  (bools), `remoteActive` (bool — when false, the local driver's request is used).
- **`/skills/status/`** (outbound): `scoring`, `intake` (`ok`|`running`|`fail`), `activeSkill`,
  `phase`.
- **`/state/`** (outbound): `aimed`, `distanceToHub`, `turretTargetRot`, `hasPose` (pose estimation
  is gated off locally — `VisionConstants.kTransformsMeasured = false`).

## Standalone operation

`LocalSkillDriver` keeps the robot fully functional with no coprocessor: it maps the operator
controller (RT=shoot, LB=pass, right-stick=unjam, A=toggle intake, Start=manual, LT=feed, RB=re-zero
turret, manual POV/X/Y/B presets) to skill requests + direct manual mechanism control, and registers
the PathPlanner named commands (`intakeOut`, `intakeIn`, `shoot`, `autoShoot`, `feedIndexers`,
`stopAll`, `reverseIndexer` — exact strings the `.auto` files reference).
