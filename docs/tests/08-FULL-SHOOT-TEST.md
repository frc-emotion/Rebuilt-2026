# Test 8: Full Integration Shoot Test

**Goal**: End-to-end test — robot auto-aims at hub from multiple field positions and scores using the interpolation tables. No manual intervention except pulling the trigger.

**Time**: ~20 minutes

**Prerequisites**: Tests 0-7 ALL passed. Interp tables calibrated. On a field or large space with the hub.

---

## What this test validates (all at once)

- Turret auto-tracks DS-facing hub tag from any scoring position
- Distance reads correctly from different angles/ranges
- Interp table outputs correct hood + shooter values
- ShootCommand gates indexers until turret is aimed
- Balls score consistently

---

## Telemetry to watch

Have a second person monitoring Elastic dashboard:
- `TurretAutoAimCommand/state` — must be "TRACKING" before pulling trigger
- `TurretAutoAimCommand/visionTxDeg` — must be < 1° before balls feed
- `TurretAutoAimCommand/distanceToHubMeters` — sanity check against estimated range
- `Shooter/shooterVelocityRPS` vs `shooterSetpointRPS` — at speed before feeding?
- `Hood/hoodPositionRot` — correct angle for this distance?

---

## Test positions

Place robot at each position, let turret auto-aim, then shoot:

### Position 1: Dead center, 2.5m
- Robot directly in front of hub, bumper 2.5m from hub
- Turret should aim nearly straight (position ~0.0)
- Easiest shot — if this fails, something fundamental is wrong

### Position 2: 30° offset, 3m
- Robot 30° to the right of hub center, 3m away
- Turret should rotate left to face hub
- Tests that tracking works at an angle

### Position 3: 45° offset, 4m
- Robot far right, 4m away
- Turret needs to rotate significantly
- Tests turret range + distance at angle

### Position 4: Far range, 6m, centered
- Robot straight back, 6m from hub
- Tests long-range interp table accuracy
- Hood should be near maximum, shooter at high speed

### Position 5: Close range, 1.5m, slight offset
- Very close, slightly off-center
- Tests close-range interp values
- Hood should be nearly flat

## Procedure at each position

1. **Drive** robot to position, stop
2. **Wait** for state = "TRACKING" and tx < 2°
3. **Verify** distance reads sensible for your position
4. **Hold right trigger** — shooter spins up, hood adjusts
5. **Wait** ~1s for shooter to reach speed
6. **Indexers fire** when turret is aimed (tx < 1°)
7. **Observe** ball trajectory — score or miss?
8. **Record** results

### Scoring sheet

| Position | Distance (Elastic) | tx at fire | Shots | Scored | Notes |
|---|---|---|---|---|---|
| 1: center 2.5m | ___ m | ___ ° | 3 | ___ | |
| 2: 30° 3m | ___ m | ___ ° | 3 | ___ | |
| 3: 45° 4m | ___ m | ___ ° | 3 | ___ | |
| 4: center 6m | ___ m | ___ ° | 3 | ___ | |
| 5: close 1.5m | ___ m | ___ ° | 3 | ___ | |

---

## Debugging missed shots

### Balls go left or right consistently
- `visionTxDeg` at moment of fire — was it actually < 1°?
- If tx was large, indexers shouldn't have fired. Check `isAimed()` logic.
- If tx was small but balls go sideways → mechanical alignment issue, or `TURRET_AIM_OFFSET` needed

### Balls fall short
- `shooterVelocityRPS` at fire — had it reached setpoint?
- If not at setpoint, ShootCommand shouldn't gate? Check — ShootCommand doesn't gate on shooter speed, only on `isAimed()`. Consider adding a speed gate.
- If at setpoint but still short → interp table speed too low at this distance

### Balls go over
- Hood angle too high or shooter speed too high for this distance
- Check `distanceToHubMeters` — if reading lower than actual, interp table thinks robot is closer → outputs less hood/speed → wait, that would be short. If reading HIGHER than actual → outputs more hood/speed → could overshoot.

### Balls score at center but miss at angles
- Distance may be off when shooting at an angle (camera sees tag at an angle, perspective changes)
- This is a known limitation of single-tag distance. Consider averaging or adding more tolerance.

### Balls score at some distances but not others
- Interp table needs more data points in the failing range
- Go back to Test 7 and add a calibration point at the failing distance

---

## Pass criteria
- [ ] 2/3 balls score at each of the 5 positions (10/15 minimum)
- [ ] Turret auto-tracks at all positions without manual intervention
- [ ] Distance telemetry is reasonable at each position
- [ ] No turret soft limit faults during any test
- [ ] Shooter reaches setpoint before balls fire at every position
