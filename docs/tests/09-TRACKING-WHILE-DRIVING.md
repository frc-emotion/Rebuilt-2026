# Test 9: Tracking While Driving — Gyro Feedforward

**Goal**: Verify the turret maintains lock on the hub while the robot is translating and rotating. Enable and tune the gyro feedforward to pre-compensate for robot yaw.

**Time**: ~15 minutes

**Prerequisites**: Tests 0-8 ALL passed. Shooting works while stationary. On a field with space to drive.

---

## Placeholder values resolved in this test

| Value | Current | What to set |
|---|---|---|
| `GYRO_FF_ENABLED` | false | Set to `true` once basic tracking is confirmed while driving |
| `LOOP_PERIOD_SEC` | 0.02 | Should match actual loop period (50 Hz = 0.02s). Don't change unless running a different loop rate. |

---

## How gyro feedforward works

Without gyro FF:
```
Robot rotates 5°/s CCW → turret target stays fixed →
camera sees tag drift right → vision corrects 1 frame later (~40ms) →
turret chases, always 1 frame behind
```

With gyro FF:
```
Robot rotates 5°/s CCW → gyro detects instantly →
turret target adjusts CW by (5/360 × 0.02) = 0.000278 rot per cycle →
turret pre-compensates → vision only fixes the residual error
```

The gyro runs at 1kHz (read every 20ms in our loop) — much faster than camera frames (15-30 Hz). This fills the gap between camera updates.

---

## Telemetry to watch

- `TurretAutoAimCommand/visionTxDeg` — **the key metric**. Should stay small even while driving.
- `TurretAutoAimCommand/gyroFeedforwardRot` — per-cycle FF adjustment (only non-zero when GYRO_FF_ENABLED = true)
- `TurretAutoAimCommand/turretErrorRot` — motor tracking error
- `TurretAutoAimCommand/state` — must stay "TRACKING" throughout
- `Vision/timeSinceLastFrameMs` — if this spikes, camera is dropping frames during motion

---

## Phase 1: Driving WITHOUT gyro FF (baseline)

1. Keep `GYRO_FF_ENABLED = false`
2. Drive the robot slowly (~0.5 m/s) while the turret tracks the hub
3. Watch `visionTxDeg` — record the peak tx during motion
4. Drive in a slow circle around the hub at ~3m
5. Record: Does `state` ever drop to MANUAL during the circle? How large does tx get?

### Expected behavior without FF
- tx will oscillate ±5-10° during smooth driving
- During sharp turns, tx may spike to ±15° or more
- The turret "chases" with 1-2 frame delay
- State may briefly drop to MANUAL if the tag leaves the camera FOV during fast turns

## Phase 2: Enable gyro FF

1. Set `GYRO_FF_ENABLED = true` in `TurretAutoAimCommand.java` line 87
2. Deploy and re-enable

## Phase 3: Driving WITH gyro FF

1. Repeat the same driving patterns as Phase 1
2. Watch `visionTxDeg` — should be noticeably smaller during motion
3. Watch `gyroFeedforwardRot` — should pulse with each cycle during turns
4. Drive in a slow circle around the hub

### Expected improvement
- tx should stay within ±3-5° during smooth driving (vs ±5-10° without FF)
- During turns, tx should recover faster
- State should drop to MANUAL less often

## Phase 4: Sign verification

This is critical — if the gyro FF sign is wrong, it will **double** the error instead of reducing it.

1. Drive the robot in a **slow clockwise circle** around the hub
2. If tx gets **worse** (larger) than without FF → sign is wrong
3. Fix: In `TurretAutoAimCommand.java` line 225, flip the sign:
   ```java
   // Current:
   gyroFeedforwardRot = (gyroRateDegPerSec / 360.0) * LOOP_PERIOD_SEC;
   // If wrong sign:
   gyroFeedforwardRot = -(gyroRateDegPerSec / 360.0) * LOOP_PERIOD_SEC;
   ```

## Phase 5: Shoot while driving (stretch goal)

1. Drive slowly toward/away from hub while turret tracks
2. Hold right trigger to shoot
3. Balls should still score if driving slowly (<0.5 m/s)
4. At higher speeds, scoring rate will drop — this is expected

---

## Debugging

| Symptom | Fix |
|---|---|
| tx gets WORSE with FF enabled | Flip the sign of gyroFeedforwardRot |
| Turret oscillates rapidly with FF | gyroRateDegPerSec is noisy — add a low-pass filter or reduce the feedforward gain |
| gyroFeedforwardRot is always 0 | Check `GYRO_FF_ENABLED` is true, check `drivetrain != null` |
| State drops to MANUAL during drives | Camera losing tag — drive slower, check if turret is hitting soft limits |
| tx is small but turretErrorRot is large | MotionMagic can't keep up — turret is being commanded faster than it can move. Increase CruiseVelocity. |

---

## Pass criteria
- [ ] Baseline (no FF): tx behavior recorded during driving
- [ ] GYRO_FF_ENABLED set to true, deployed
- [ ] With FF: tx is measurably smaller during motion than baseline
- [ ] Sign is correct (FF reduces tx, not increases it)
- [ ] State stays TRACKING during slow circles around hub
- [ ] (Stretch) Balls score while driving slowly at 2-3m range
