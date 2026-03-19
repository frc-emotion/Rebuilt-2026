# Test 6: Turret Tracking — Lock-On, Convergence, Tuning

**Goal**: Verify the turret automatically locks onto a hub AprilTag, smoothly converges tx toward zero, and holds aim under small disturbances. This is where the tracking control loop gets tuned.

**Time**: ~15 minutes

**Prerequisites**: Tests 0-5 passed. Distance reads correctly. AprilTag at ~2-3m.

---

## Placeholder values resolved in this test

| Value | Current | What to set |
|---|---|---|
| `TX_TO_ROT_GAIN` | 1.0 | Tune: 1.0 = 1:1 tx-to-rotation conversion |
| `DEADBAND_DEG` | 1.0 | Tune: how close to center before we stop correcting |
| `TURRET_AIM_OFFSET` | 0.0 | Set if turret consistently aims off-center |

---

## How tracking works (understand this first)

Each camera frame:
1. Camera reports `tx` = degrees the tag is off-center (+ = tag is right of camera center)
2. Code converts: `turret_correction = -(tx / 360) * TX_TO_ROT_GAIN`
3. Correction is **added** to `persistentTargetRot`
4. MotionMagic moves turret to new target
5. Next frame, tx should be smaller
6. Repeat until `|tx| < DEADBAND_DEG`

The negative sign is because if the tag is to the **right** (positive tx), the turret must rotate **right** (positive rotation in CW_Positive convention) to center it — but the camera moves with the turret, so the tag appears to move left.

---

## Telemetry to watch

- `TurretAutoAimCommand/visionTxDeg` — **the key metric**. Should converge to ~0.
- `TurretAutoAimCommand/turretErrorRot` — motor tracking error (separate from vision)
- `TurretAutoAimCommand/targetPositionRot` — where the code wants the turret
- `TurretAutoAimCommand/currentPositionRot` — where the turret actually is
- `TurretAutoAimCommand/state` — must be "TRACKING"
- `TurretAutoAimCommand/trackedTagId` — which tag

---

## Step 1: Basic lock-on

1. Place an AprilTag at ~2m, visible to the turret camera
2. Enable robot in teleop
3. State should switch to "TRACKING"
4. Watch `visionTxDeg` — it should decrease toward 0
5. The turret should physically rotate to center the tag

### If turret moves AWAY from the tag
**The sign is wrong.** The `TX_TO_ROT_GAIN` correction direction is inverted for your motor convention.

Fix in `TurretAutoAimCommand.java` line 234:
```java
// Current: double txOffsetRot = -(visionTxDeg / 360.0) * TX_TO_ROT_GAIN;
// If inverted, flip the sign:
double txOffsetRot = (visionTxDeg / 360.0) * TX_TO_ROT_GAIN;
```

### If turret doesn't move at all in TRACKING
- Check `visionTxDeg` — is it updating? If stuck at 0.0, the camera may not be providing tx.
- Check `freshVisionThisCycle` — if this is always false, frames aren't being recognized as fresh.
- Check `trackedTagId` — is it -1? Then the tag isn't being accepted by the DS-facing filter.

## Step 2: Convergence speed tuning

With tag at ~2m, offset turret by ~10° using D-pad, then release:

1. The turret should re-acquire the tag and converge
2. Time how long until `|visionTxDeg| < 1.0°`
3. Target: **< 1 second** for 10° offset

### If convergence is too slow (>2s)
- `TX_TO_ROT_GAIN` is too low → increase from 1.0 to 1.5 or 2.0
- Each camera frame adds a larger correction

### If turret overshoots and oscillates around the tag
- `TX_TO_ROT_GAIN` is too high → decrease from 1.0 to 0.7 or 0.5
- Or `DEADBAND_DEG` is too small → increase from 1.0 to 2.0 (stops correcting sooner)

### If turret settles but not perfectly centered
- `DEADBAND_DEG` might be too large → try 0.5°
- Or there's a consistent offset → set `TURRET_AIM_OFFSET` in TurretConstants

## Step 3: Offset test

If the turret consistently aims slightly left or right of the tag (tx settles at e.g. +2° instead of 0°):

1. Note the consistent offset in degrees
2. Convert to rotations: `offset_rot = offset_deg / 360`
3. Set `TURRET_AIM_OFFSET` in TurretConstants to compensate
4. This is applied to all turret setpoints

## Step 4: Disturbance rejection

With the turret locked on:
1. Gently push the turret ~5° off target by hand (careful — let the motor fight you)
2. Release — turret should snap back to aiming at the tag
3. Watch `visionTxDeg` — should return to <1° within 0.5s

## Step 5: Moving tag test

1. Hold the tag and slowly walk left/right in front of the robot (stay at ~2m distance)
2. The turret should smoothly follow the tag
3. `visionTxDeg` should stay within ±3° during slow movement
4. If tx oscillates wildly during movement → reduce `TX_TO_ROT_GAIN`

## Step 6: Edge of range test

1. Move tag to ~5m distance
2. Tracking should still work but tx convergence may be slower (tag appears smaller)
3. Move tag to ~1m — tracking should work, tx may jump more (tag is large, small movements = big tx changes)

---

## Debugging table

| Symptom | Telemetry clue | Fix |
|---|---|---|
| Turret moves wrong direction | tx goes UP when turret moves | Flip sign in txOffsetRot calculation |
| Turret oscillates around tag | tx alternates +/- each frame | Reduce TX_TO_ROT_GAIN or increase DEADBAND_DEG |
| Turret tracks but offset | tx settles at constant non-zero value | Set TURRET_AIM_OFFSET |
| Turret hunts slowly | tx decreases very slowly | Increase TX_TO_ROT_GAIN |
| Turret tracks then loses tag | timeSinceLastFrameMs spikes | Camera latency issue — see Test 3 |
| Turret jitters at setpoint | turretErrorRot oscillates | Motor PID issue — revisit Test 1 |

---

## Pass criteria
- [ ] Turret locks onto tag and centers it (tx < 1°) within 1 second
- [ ] No overshoot oscillation during lock-on
- [ ] TX_TO_ROT_GAIN tuned and recorded: ___
- [ ] DEADBAND_DEG tuned and recorded: ___
- [ ] Turret follows slowly moving tag smoothly
- [ ] Disturbance rejection works (snaps back after push)
- [ ] TURRET_AIM_OFFSET set if needed: ___
