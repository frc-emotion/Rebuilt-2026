# Driver & Operator Controls — Complete Reference

Both controllers are Xbox-style gamepads. Driver = port 0, Operator = port 1.

---

## Controller Layout Reference

```
              [LB]                    [RB]
              [LT]                    [RT]
         ┌─────────────────────────────────┐
         │         [Back]  [Start]         │
         │                                 │
         │    [L-Stick]         [R-Stick]  │
         │      ↕ ↔              ↕ ↔       │
         │                                 │
         │   [D-pad]     [Y]              │
         │    ↑                            │
         │  ← + →      [X] [B]            │
         │    ↓                            │
         │              [A]                │
         └─────────────────────────────────┘
```

LT/RT = analog triggers (0.0 to 1.0)
LB/RB = bumper buttons (digital)
L-Stick / R-Stick = analog sticks (click = button)

---

## Driver Controller (Port 0) — Driving + Utilities

The driver handles robot movement and has utility/diagnostic functions.

### Sticks

| Input | Action | Details |
|---|---|---|
| **Left stick Y** | Drive forward/backward | Field-centric. Full range = max speed. 10% deadband. |
| **Left stick X** | Drive strafe left/right | Field-centric. Full range = max speed. 10% deadband. |
| **Right stick X** | Rotate robot | Field-centric. 0.75 rot/s max angular rate. 10% deadband. |

### Buttons

| Input | Type | Action | Details |
|---|---|---|---|
| **A** | Hold | **Brake** | Locks all 4 swerve modules in X-pattern. Robot stops instantly and resists pushing. |
| **B** | Hold | **Calibration shoot** | Holds turret at center, hood + shooter speed from NetworkTables. All indexers fire. For tuning interp tables. |
| **Y** | Press | **Seed pose** | Sets odometry to ~4m in front of red hub, facing +X. Useful for testing without full field. |
| **Left bumper** | Press | **Seed field-centric** | Resets the gyro heading so "forward" = current robot facing direction. Use if field-centric driving is wrong. |

### D-pad (TEMPORARY — SysId characterization)

| Input | Action |
|---|---|
| **D-pad Up** | SysId quasistatic forward |
| **D-pad Down** | SysId quasistatic reverse |
| **D-pad Right** | SysId dynamic forward |
| **D-pad Left** | SysId dynamic reverse |

These are for drive motor characterization. Remove after tuning is complete.

### Default behavior
When no buttons are pressed, the drivetrain responds to the sticks with field-centric swerve drive. When the robot is disabled, all modules go idle.

---

## Operator Controller (Port 1) — Superstructure

The operator controls everything above the drivetrain: turret, hood, shooter, intake, indexers.

### Sticks

| Input | Action | Details |
|---|---|---|
| **Right stick X** | **Manual turret control** | Only active when turret is in MANUAL state (no hub tag visible). Deadband 0.08. Applies direct voltage to turret motor (±3V max). Ignored when TRACKING. |

### Triggers

| Input | Type | Action | Details |
|---|---|---|---|
| **Right trigger** | Hold | **SHOOT** | If turret is TRACKING → vision shot: hood + shooter from interp tables, indexers gated on `isAimed()`. If turret is MANUAL → manual shot: shooter at 40 RPS, hood flat, indexers always run. |
| **Left trigger** | Hold | **Vertical indexer only** | Runs just the vertical indexer at 50 RPS. For manually staging balls without firing. Stops on release. |

### Bumpers

| Input | Type | Action | Details |
|---|---|---|---|
| **Right bumper** | Press | **Zero turret** | Sets turret motor position to 0 at current physical position. Use if turret was bumped and 0.0 no longer means "forward." |
| **Left bumper** | — | **Unused** | Intake irrigate is coded but commented out. Could be re-enabled for dislodging stuck balls. |

### Face buttons

| Input | Type | Action | Details |
|---|---|---|---|
| **A** | Press | **Intake toggle** | If intake is out → stow (pivot to 0.3 rot, roller stops). If intake is in → deploy (pivot to 0.653 rot, roller starts at 55 RPS). |
| **X** | Hold | **Hood down** | Moves hood to 0.005 rot (1.8°). Overrides default hold-position while pressed. |
| **Y** | Hold | **Hood mid** | Moves hood to 0.040 rot (14.4°). |
| **B** | Hold | **Hood up** | Moves hood to 0.070 rot (25.2°). |

### D-pad — Turret presets

| Input | Type | Action | Position |
|---|---|---|---|
| **D-pad Up** | Hold | **Turret center** | 0.0 rot (straight forward) |
| **D-pad Right** | Hold | **Turret right** | +0.15 rot (~54° CW) |
| **D-pad Left** | Hold | **Turret left** | -0.20 rot (~72° CCW) |
| **D-pad Down** | Hold | **Turret far left** | -0.45 rot (~162° CCW) |

These **interrupt** the default TurretAutoAimCommand while held. When released, the turret returns to auto-aim (MANUAL joystick or TRACKING vision, depending on tag visibility).

### Stick clicks

| Input | Type | Action | Details |
|---|---|---|---|
| **Right stick click** | Hold | **Reverse indexers** | All three indexers run in reverse at 50% speed. For clearing jams. Stops on release. |
| **Left stick click** | — | **Unused** | Mode toggle was removed. Turret auto-switches between MANUAL and TRACKING. |

### Default behaviors (always running)

| Subsystem | Default command | What it does |
|---|---|---|
| **Turret** | `TurretAutoAimCommand` | MANUAL state: operator right stick X controls turret. TRACKING state: vision auto-aims at hub tag. Always running. |
| **Hood** | Hold position | Continuously commands MotionMagic to the current position, preventing drift. |

---

## Interaction Between Controls

### Turret control priority (highest to lowest)
1. **D-pad preset** (operator) — interrupts everything while held
2. **TurretAutoAimCommand TRACKING** — vision controls turret when a hub tag is visible
3. **TurretAutoAimCommand MANUAL** — operator right stick X when no tag is visible

When a D-pad button is released, control returns to TurretAutoAimCommand automatically.

### Shoot behavior depends on turret state

| Turret state | Right trigger behavior |
|---|---|
| **TRACKING** (tag visible) | Hood + shooter from interp table at detected distance. Indexers only fire when `isAimed()` (tx < 1°). |
| **MANUAL** (no tag) | Shooter at fixed 40 RPS. Hood set to 0 (flat). Indexers fire immediately (no aim gate). |

### Subsystem conflicts — what CAN'T run simultaneously

Commands that require the same subsystem cancel each other:

| Subsystem | Commands that use it |
|---|---|
| Turret | TurretAutoAimCommand, D-pad presets, CalibrationShootCommand |
| Hood | Default hold, X/Y/B presets, ShootCommand, CalibrationShootCommand |
| Shooter | ShootCommand, CalibrationShootCommand |
| Indexer | ShootCommand, left trigger, right stick click, runIndexer |
| Intake | A toggle, IntakeIrrigateCommand |

**Example conflict**: Pressing driver B (CalibrationShoot) while operator holds right trigger (ShootCommand) — CalibrationShoot requires turret, hood, shooter, and indexer, so it would cancel ShootCommand.

---

## Quick Reference Card

```
╔══════════════════════════════════════════════════════════╗
║                    DRIVER (Port 0)                       ║
║                                                          ║
║  L-Stick: Drive (field-centric)   R-Stick: Rotate        ║
║  A: Brake    B: Calibration shoot    Y: Seed pose        ║
║  LB: Seed heading    D-pad: SysId (temporary)            ║
╠══════════════════════════════════════════════════════════╣
║                   OPERATOR (Port 1)                      ║
║                                                          ║
║  R-Stick X: Manual turret      RT: SHOOT                 ║
║  LT: Vertical indexer          RB: Zero turret            ║
║  A: Intake toggle                                        ║
║  X: Hood down  Y: Hood mid  B: Hood up                   ║
║  D-pad: Turret presets (Up=center, R=right, L=left,      ║
║         Down=far left)                                   ║
║  R-Stick click: Reverse indexers (clear jams)            ║
╚══════════════════════════════════════════════════════════╝
```
