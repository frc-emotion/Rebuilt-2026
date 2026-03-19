# Test 1: Turret Position Control & PID Tuning

**Goal**: Verify turret moves to commanded positions accurately, soft limits work, and PID is tuned.

**Time**: ~20 minutes

**Prerequisites**: Test 0 passed. Turret facing straight forward at power-on.

---

## Values to validate (not change — test existing first)

| Value | Current | Notes |
|---|---|---|
| Turret `kS` | 0.0 | **Intentionally zero** — turret cable routing causes uneven, unpredictable friction. A static kS would be wrong at some positions. The PID loop handles friction on its own. |
| Turret `kP` | 20.0 | Test as-is. Only change if oscillation or steady-state error is observed. |
| Turret `kI` | 5.0 | Test as-is. Helps push through cable friction. |
| MotionMagic CruiseVelocity | 1.0 RPS | Test as-is. |
| MotionMagic Acceleration | 4.0 RPS² | Test as-is. |

---

## Telemetry to watch

Add these to Elastic:
- `Turret/turretPositionRot` — actual position
- `Turret/turretSetpointRot` — commanded position
- `Turret/turretErrorRot` — difference (should → 0)
- `Turret/turretVelocityRPS` — speed during moves
- `Turret/turretVoltageVolts` — motor effort
- `Turret/faultForwardSoftLimit` — should be false unless at limit
- `Turret/faultReverseSoftLimit` — should be false unless at limit

---

## Step 1: Verify zero position

1. Confirm turret is physically pointing straight forward
2. Enable robot in teleop
3. Check `Turret/turretPositionRot` = ~0.0
4. If not 0.0, press **operator right bumper** to re-zero

## Step 2: Test D-pad presets

Command each preset and observe the response:

| Button | Expected position | Watch for |
|---|---|---|
| D-pad Up | 0.0 (center) | Already there — no movement |
| D-pad Right | +0.15 | Smooth move right, settles within 0.5s |
| D-pad Left | -0.20 | Smooth move left |
| D-pad Down | -0.45 | Long move to far left |

For each move, record in Elastic:
1. **Rise time**: How long until `turretErrorRot` < 0.01?
2. **Overshoot**: Does position go past setpoint then come back?
3. **Steady-state error**: What is `turretErrorRot` after 2 seconds?
4. **Peak voltage**: What's the max `turretVoltageVolts` during the move?

## Step 3: Interpret results and tune (only if needed)

### If turret oscillates around setpoint (bounces back and forth)
- **Cause**: kP too high or kI winding up
- **Fix**: Reduce kP from 20 to 15 (via SuperstructureTuner). If still oscillating, reduce kI from 5 to 2.

### If turret doesn't reach setpoint (steady-state error > 0.005)
- **Cause**: kP too low or friction not compensated
- **Fix**: Verify kS is set correctly. If error persists, increase kI slightly.

### If turret slams to position (violent snap)
- **Cause**: MotionMagic acceleration too high
- **Fix**: Reduce `MotionMagicAcceleration` from 4.0 to 2.0. Reduce `CruiseVelocity` from 1.0 to 0.5.

### If turret is sluggish (takes >1s to reach setpoint)
- **Cause**: CruiseVelocity or Acceleration too low
- **Fix**: Increase CruiseVelocity from 1.0 to 2.0. Increase Acceleration from 4.0 to 8.0.

### If motor voltage hits 10V and turret still isn't reaching
- **Cause**: kP too high (commanding more than available), or mechanical binding
- **Fix**: Check mechanism for binding. Reduce kP if needed.

## Step 4: Soft limit verification

1. Push **operator right stick X** fully right — turret should stop at +0.20
2. Check `faultForwardSoftLimit` = **true** in Elastic
3. Push fully left — turret should stop at -0.58
4. Check `faultReverseSoftLimit` = **true**
5. Both faults should clear when you release the stick

### If turret goes past soft limits
**CRITICAL**: Stop immediately. Power off. The turret could hit the intake.
- Check that `TURRET_CONFIG` soft limit thresholds match code (0.20 forward, -0.58 reverse)
- Check that `ForwardSoftLimitEnable = true` and `ReverseSoftLimitEnable = true`
- Re-verify the console readback from Test 0

## Step 5: Record values (only if you changed anything)

If the existing PID worked fine out of the box, skip this — don't fix what isn't broken.

If you had to tune via SuperstructureTuner, record the working values and update `TurretConstants.java`:

```
kP = ___
kI = ___
kD = ___
CruiseVelocity = ___ RPS
Acceleration = ___ RPS²
Jerk = ___
```

---

## Pass criteria
- [ ] Turret reaches all 4 D-pad presets within 1 second
- [ ] Steady-state error < 0.005 rot (< 1.8°)
- [ ] No overshoot greater than 0.01 rot
- [ ] Soft limits stop turret at both ends
- [ ] Peak voltage stays under 10V during all moves
