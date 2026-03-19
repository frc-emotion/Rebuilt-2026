# Test 1B: Power & Current Draw Verification

**Goal**: Verify current limits are working, no motors are stalling or drawing excessive current, and the robot doesn't brownout during a full shooting cycle.

**Time**: ~10 minutes

**Prerequisites**: Test 0 passed. Robot enabled in teleop.

---

## Why this matters

Every motor has stator and supply current limits configured in firmware. If a limit is set too high, the motor could trip a breaker, brownout the robot, or damage a mechanism. If too low, the motor won't have enough torque to function.

## Current limit reference

| Motor | Stator | Supply | Peak Voltage | Breaker (typical) |
|---|---|---|---|---|
| Shooter | 120A | 60A | 10V / 0V | 40A |
| Turret | 80A | 40A | ±10V | 30A |
| Hood | 40A | 20A | ±10V | 20A |
| Intake pivot | 80A | 30A | ±10V | 30A |
| Intake roller | 60A | 30A | ±10V | 30A |
| Each indexer (×3) | 60A | 30A | ±10V | 30A |
| Climb leader | 80A | 60A | ±10V | 40A |
| Climb follower | 80A | 60A | ±10V | 40A |

**Note**: Supply current limit should always be at or below the breaker rating, otherwise the breaker trips before the limit kicks in.

---

## Telemetry to watch

For each motor, check supply current in Elastic or Phoenix Tuner X:
- `Turret/turretCurrentAmps`
- `Hood/hoodCurrentAmps`
- `Shooter/shooterCurrentAmps` (this is supply current)
- `Intake/pivotCurrentAmps`

For motors without dedicated telemetry fields, use Phoenix Tuner X to plot `SupplyCurrent` directly.

Also monitor:
- **Battery voltage** (via Driver Station or PDP/PDH telemetry) — should stay above 7V during all operations

---

## Step 1: Idle current check

With robot enabled but nothing moving:
1. All motor currents should be near 0A (< 2A)
2. Battery voltage should be stable at 12+ V
3. If any motor draws significant current at idle, it's fighting something (binding, wrong neutral mode, stuck against hard stop)

## Step 2: Individual mechanism current

Test each mechanism alone and record peak current:

### Turret
1. Command D-pad Right (+0.15) from center
2. Watch `turretCurrentAmps` during the move
3. **Expected**: Peak 5-15A during acceleration, < 2A when holding position
4. **Problem if**: Sustained > 30A = binding or stall. > 40A = supply limit should kick in.

### Hood
1. Command B (0.070) from 0
2. **Expected**: Peak 2-5A, < 1A holding
3. **Problem if**: > 15A sustained = gear binding

### Shooter
1. Hold operator RT (40 RPS manual)
2. **Expected**: 10-20A during spin-up, 3-8A steady state
3. Watch `shooterVoltageVolts` — should be 4-5V at steady state (40 RPS)
4. **Problem if**: > 40A sustained = something rubbing the flywheel

### Intake
1. Press A to deploy
2. **Expected**: Peak 10-20A during pivot, < 5A when roller running
3. **Problem if**: > 30A sustained = pivot binding or hitting something

### Indexers
1. Hold operator RT to fire indexers
2. **Expected**: 3-8A each
3. **Problem if**: > 20A on any indexer = ball jam or mechanical binding

## Step 3: Full shoot cycle — brownout test

This is the big one. Everything runs simultaneously:

1. Deploy intake (A) — intake pivot + roller running
2. Let turret track a tag (state = TRACKING)
3. Hold operator RT — shooter spins up + hood moves + indexers fire
4. **Watch battery voltage** — it will dip during shooter spin-up
5. Feed 2-3 balls through

### Battery voltage expectations
- **Normal dip**: 12V → 10-11V during shooter spin-up (< 2V sag)
- **Concerning**: Dips below 8V — may cause CAN timeouts
- **Brownout**: Dips below 6.75V — roboRIO disables all outputs

### If brownout occurs
- Check battery health with a Battery Beak (internal resistance should be < 15 mΩ)
- Reduce shooter stator current limit from 120A to 80A (slower spin-up but less draw)
- Stagger operations: spin up shooter BEFORE deploying intake
- Check all power connections (battery leads, PDP terminals)

## Step 4: Sustained operation test

1. Shoot 5+ balls in quick succession
2. Monitor for breaker trips (sudden motor stop + fault)
3. If a breaker trips → the supply current limit for that motor is higher than the breaker rating. Reduce the supply limit in code.

### Supply limit vs breaker guide
The supply current limit should be **at or below** the breaker rating. If the breaker is 30A, the supply limit should be ≤ 30A. Current limits that exceed breaker ratings:

| Motor | Supply Limit | Likely Breaker | Status |
|---|---|---|---|
| Shooter | 60A | 40A | **CHECK** — may trip 40A breaker |
| Turret | 40A | 30A | **CHECK** — may trip 30A breaker |
| Climb | 60A | 40A | **CHECK** — may trip 40A breaker |

**Action**: Verify which breaker each motor is on and reduce supply limits if needed.

---

## Pass criteria
- [ ] No motors draw > 5A at idle
- [ ] Each mechanism's peak current is within expected range
- [ ] Full shoot cycle doesn't brownout (battery stays > 8V)
- [ ] No breakers trip during 5-ball sustained shooting
- [ ] Supply current limits verified to be ≤ breaker ratings
