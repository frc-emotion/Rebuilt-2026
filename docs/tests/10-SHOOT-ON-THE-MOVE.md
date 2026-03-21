# Test 10: Shoot-On-The-Move (SOTM)

**Goal**: Verify the robot can score FUEL while driving, using lead angle, effective distance, and shooter speed compensation.

**Time**: ~30 minutes

**Prerequisites**: Tests 0-9 ALL passed. Stationary shooting works at 2-5m. Gyro feedforward verified (Test 9). On a field with space to drive circles around the hub.

---

## Placeholder values resolved in this test

| Value | Current | What to set |
|---|---|---|
| `SOTM_ENABLED` | false | Set to `true` after Phase 1 baseline |
| `BALL_EXIT_EFFICIENCY` | 0.7 | Tune if shots consistently over/under |
| `MAX_SOTM_SPEED_MPS` | 2.0 | Lower if shots are inaccurate at high speed |
| `SOTM_AIM_TOLERANCE_DEG` | 3.0 | Tighten if balls spray, loosen if indexers rarely fire |

---

## Telemetry to watch

- `TurretAutoAimCommand/leadAngleRot` — should be non-zero during strafing
- `TurretAutoAimCommand/effectiveDistanceMeters` — should differ from `distanceToHubMeters` when driving radially
- `TurretAutoAimCommand/shooterRPSOffset` — should be positive when retreating, negative when approaching
- `TurretAutoAimCommand/robotSpeedMPS` — robot speed in field frame
- `TurretAutoAimCommand/visionTxDeg` — should stay small even while moving
- `TurretAutoAimCommand/state` — must stay "TRACKING" throughout

---

## Phase 1: Stationary baseline (SOTM disabled)

1. Keep `SOTM_ENABLED = false`
2. Shoot 10 balls from 3m stationary → record hit rate
3. Shoot 10 balls from 5m stationary → record hit rate
4. These are your baseline numbers

## Phase 2: Moving baseline (SOTM disabled)

1. Still `SOTM_ENABLED = false`
2. Drive slowly (~0.5 m/s) perpendicular to the hub at ~3m while shooting
3. Record hit rate out of 10 balls
4. Drive slowly (~0.5 m/s) toward/away from hub while shooting
5. Record hit rate out of 10 balls

### Expected behavior without SOTM
- Strafing shots: balls miss to the side the robot came from (~30-50% hit rate)
- Approaching shots: balls go over (too much speed)
- Retreating shots: balls fall short (not enough speed)

## Phase 3: Enable SOTM

1. Set `SOTM_ENABLED = true` in `TurretAutoAimCommand.java` line 101
2. Deploy and re-enable

## Phase 4: SOTM strafing test

1. Drive slowly (~0.5 m/s) perpendicular to hub at ~3m while shooting
2. Watch `leadAngleRot` — should be non-zero and change sign when reversing direction
3. Record hit rate out of 10 balls — should improve over Phase 2

### Expected improvement
- Strafing: hit rate should improve to ~60-80%
- `leadAngleRot` should be ~0.001-0.003 rot at 0.5 m/s

## Phase 5: SOTM radial test

1. Drive slowly (~0.5 m/s) toward hub while shooting at ~3-4m
2. Watch `effectiveDistanceMeters` — should be less than `distanceToHubMeters`
3. Watch `shooterRPSOffset` — should be slightly negative
4. Drive slowly away from hub while shooting
5. Watch `effectiveDistanceMeters` — should be more than `distanceToHubMeters`
6. Record hit rate

## Phase 6: Lead angle sign verification (CRITICAL)

1. Drive robot in a slow clockwise strafe past the hub at ~3m
2. Note where balls land relative to the hub opening:
   - **Balls land AHEAD of hub** (in the direction you came from) → sign is WRONG
   - **Balls land IN the hub or slightly behind** → sign is CORRECT
3. If wrong, fix in `TurretAutoAimCommand.java` line 392:
   ```java
   // Current:
   double vTangential = -vx * unitToHub.getY() + vy * unitToHub.getX();
   // If wrong sign:
   double vTangential = vx * unitToHub.getY() - vy * unitToHub.getX();
   ```

## Phase 7: Speed ramp-up

1. Gradually increase driving speed while shooting: 0.5, 1.0, 1.5, 2.0 m/s
2. Record hit rate at each speed
3. If hit rate drops below 30% at a speed, consider lowering `MAX_SOTM_SPEED_MPS`
4. At speeds above `MAX_SOTM_SPEED_MPS`, SOTM auto-disables and reverts to stationary aiming

## Phase 8: Efficiency tuning

If balls consistently fall short during all moving shots:
- `BALL_EXIT_EFFICIENCY` is too high (flight time estimate too short)
- Lower to 0.5-0.6

If balls consistently go long:
- `BALL_EXIT_EFFICIENCY` is too low (flight time estimate too long)
- Raise to 0.8-0.9

---

## Debugging

| Symptom | Fix |
|---|---|
| `leadAngleRot` always 0 | Check `SOTM_ENABLED` is true, check `robotSpeedMPS` > 0.3, check `distanceToHubMeters` > 0.5 |
| Balls miss to wrong side during strafe | Flip sign of `vTangential` (Phase 6) |
| Balls always fall short while moving | Increase `BALL_EXIT_EFFICIENCY` or check `effectiveDistanceMeters` is reasonable |
| Balls always go long while moving | Decrease `BALL_EXIT_EFFICIENCY` |
| `effectiveDistanceMeters` jumps wildly | Odometry is drifting — check wheel calibration |
| Hit rate worse with SOTM than without | Multiple signs may be wrong — test each layer separately by commenting out the others |
| Turret oscillates during movement | Lead angle delta is too large — add a rate limiter or reduce `MAX_SOTM_SPEED_MPS` |
| Indexers never fire while moving | `SOTM_AIM_TOLERANCE_DEG` too tight — increase to 5.0° |

---

## Isolating layers (if debugging)

To test each SOTM layer independently, temporarily modify `computeSOTMCompensation()`:

1. **Lead angle only**: Comment out `effectiveDistanceMeters` and `shooterRPSOffset` assignments
2. **Effective distance only**: Comment out `leadAngleRot` assignment and `shooterRPSOffset`
3. **Shooter offset only**: Comment out `leadAngleRot` and `effectiveDistanceMeters`

---

## Pass criteria
- [ ] Phase 1-2: Baseline hit rates recorded
- [ ] `SOTM_ENABLED` set to true, deployed
- [ ] Phase 4: Strafing hit rate improves over baseline
- [ ] Phase 5: Radial hit rate improves over baseline
- [ ] Phase 6: Lead angle sign is correct
- [ ] Phase 7: SOTM works at 1.0 m/s with >50% hit rate at 3m
- [ ] (Stretch) SOTM works at 1.5 m/s with >30% hit rate at 3m
