# Test 2: Hood Position & Shooter Velocity Tuning

**Goal**: Verify hood reaches angle setpoints precisely and shooter holds target speed. These must work before interp table calibration.

**Time**: ~15 minutes

**Prerequisites**: Test 0, 1 passed.

---

## Values to validate (not change — test existing first)

| Value | Current | Notes |
|---|---|---|
| Hood `kS` | 0.0 | Leave at zero — hood gear friction is high enough that PID handles it. |
| Hood `kG` | 0.0 | **Intentionally zero** — hood does not move under gravity without being commanded. Internal friction holds it. |
| Hood `kP` | 100.0 | Test as-is. High because total range is tiny (0.08 rot). |
| Hood `kI` | 50.0 | Test as-is. Pushes through gear friction. |
| `HOOD_ENCODER_OFFSET` | 0.0 | Verify: hood is at bottom hard stop on boot |

---

## Part A: Hood Position Control

### Telemetry to watch
- `Hood/hoodPositionRot` — actual position (0.0 = bottom, 0.08 = top)
- `Hood/hoodVoltageVolts` — motor effort
- `Hood/hoodVelocityRPS` — speed during moves

### Step 1: Verify zero

1. Confirm hood is physically at its bottom hard stop before power-on
2. `Hood/hoodPositionRot` should read ~0.0
3. If not, power cycle with hood at bottom stop

### Step 2: Test button presets

| Button | Expected position | Degrees |
|---|---|---|
| X | 0.005 rot | 1.8° (barely up) |
| Y | 0.040 rot | 14.4° (mid) |
| B | 0.070 rot | 25.2° (near max) |

For each:
1. Press and hold the button
2. Watch `hoodPositionRot` converge to setpoint
3. Record: settle time, steady-state error, any oscillation
4. Release — hood should hold last position (default command holds via MotionMagic)

### Step 3: Verify position hold

1. Command hood to B (0.070) and release
2. Wait 10 seconds
3. `hoodPositionRot` should stay at 0.070 (hood holds via internal friction + MotionMagic default command)
4. If it drifts, there may be a mechanical issue — but this is not expected

### Tuning troubleshooting (only if something fails above)

| Symptom | Fix |
|---|---|
| Hood oscillates | Reduce kP from 100 to 70-80. Reduce kI from 50 to 20. |
| Hood doesn't reach setpoint | Increase kI slightly via SuperstructureTuner |
| Hood slams up/down | Reduce MotionMagic Acceleration from 2.0 to 1.0 |

---

## Part B: Shooter Velocity Control

### Telemetry to watch
- `Shooter/shooterSetpointRPS` — commanded speed
- `Shooter/shooterVelocityRPS` — actual speed
- `Shooter/shooterVoltageVolts` — should stay 0-10V

### Step 1: Manual shoot test

1. Make sure NO hub tag is visible (state = MANUAL)
2. Hold **operator right trigger**
3. Shooter should spin up to 40 RPS (manual mode)
4. Watch `shooterVelocityRPS` — record:
   - **Spin-up time**: How long from 0 to 40 RPS?
   - **Steady-state error**: Difference between setpoint and actual after 2s
   - **Voltage**: Should be ~5V at 40 RPS (0.15 + 0.12 × 40 = 4.95V)

### Step 2: Release and verify coast-down

1. Release right trigger
2. `shooterVoltageVolts` should immediately drop to 0 (PeakReverseVoltage = 0)
3. Shooter should coast down gradually on friction — NOT brake
4. Watch `shooterVelocityRPS` decrease smoothly toward 0

### Step 3: Higher speed test

1. Use CalibrationShootCommand (hold **driver B**) to test higher speeds
2. In Elastic/NetworkTables, set calibration speed to 70 RPS
3. `shooterVoltageVolts` should be ~8.5V (0.15 + 0.12 × 70 = 8.55V)
4. Should stay under the 10V cap
5. If voltage hits 10V cap and speed doesn't reach setpoint → need to increase kV or accept the speed cap

### Tuning troubleshooting

| Symptom | Fix |
|---|---|
| Slow spin-up (>2s to 40 RPS) | Increase kP from 0.3 to 0.5 |
| Speed oscillates ±2 RPS | Decrease kP from 0.3 to 0.2 |
| Speed drops when ball enters | Normal — kP corrects. If recovery is too slow, increase kP. |
| Voltage hits 10V cap | Speed is near maximum. Reduce target or accept. |

---

## Pass criteria
- [ ] Hood reaches all 3 presets within 0.5s, error < 0.002 rot
- [ ] Hood holds position without drifting
- [ ] Shooter reaches 40 RPS within 1.5s
- [ ] Shooter steady-state error < 0.5 RPS
- [ ] Shooter coasts down on release (no reverse braking)
- [ ] All voltages stay under 10V
