# Test 7: Interpolation Table Calibration

**Goal**: Calibrate the hood angle and shooter speed lookup tables so the robot scores at every distance from 1m to 7m. This is the longest test and the most critical for competition performance.

**Time**: ~45-60 minutes

**Prerequisites**: Tests 0-6 passed. Distance reads accurately. Turret tracks and locks on. Actual hub or scoring target required.

---

## How the interpolation tables work

Two `InterpolatingDoubleTreeMap` tables in `TurretAimingCalculator.java`:

```
distance (meters) → hood angle (mechanism rotations, 0.0 to 0.08)
distance (meters) → shooter speed (RPS)
```

When a distance falls between two calibration points, WPILib **linearly interpolates** between them. Outside the range, it **clamps** to the nearest endpoint.

**Example**: If you calibrate:
- 2.0m → 50 RPS
- 4.0m → 60 RPS

Then at 3.0m, the table returns 55 RPS (halfway between).

---

## Current table values (calibrated 2026-03-17)

| Distance (m) | Inches | Hood (rot) | Shooter (RPS) |
|---|---|---|---|
| 1.232 | 48.5" | 0.000 | 45 |
| 2.216 | 87.25" | 0.016 | 50 |
| 3.232 | 127.25" | 0.031 | 55 |
| 4.191 | 165" | 0.046 | 60 |
| 5.207 | 205" | 0.050 | 63 |
| 6.223 | 245" | 0.053 | 69 |
| 7.239 | 285" | 0.055 | 74 |

**These may be wrong** if:
- `CAMERA_TO_BUMPER_OFFSET_METERS` changed since calibration (the distance key shifts)
- The hood or shooter mechanism was rebuilt
- Battery voltage has changed significantly

---

## Strategy: When to recalibrate vs. translate

### Option A: Full recalibration (recommended first time)
Start from scratch at every distance. Slow but most accurate.

### Option B: Distance translation (if only the offset changed)
If the only thing that changed is `CAMERA_TO_BUMPER_OFFSET_METERS`, the hood/shooter values at each *physical* distance are still correct — only the distance *key* shifted. You can translate the table:

```
new_key = old_key + (new_offset - old_offset)
```

**Example**: Old offset was 20" (0.508m), new offset is 16" (0.406m). Difference = -0.102m.
- Old: 2.216m → 50 RPS
- New: 2.216 - 0.102 = 2.114m → 50 RPS

This is a simple shift of all the distance keys. The hood and shooter values stay the same.

### Option C: Scale translation (if mechanism changed)
If the shooter was rebuilt with a different wheel or the hood gear ratio changed, you need a **multiplier** on the values, not just the keys. This requires partial recalibration — pick 2-3 distances, find the new values, compute the ratio, and apply it to all points.

---

## Full Calibration Procedure

### Equipment needed
- Tape measure (at least 25 feet / 7.5m)
- 5+ game pieces (balls)
- Actual hub or scoring target
- Elastic dashboard open with telemetry

### Setup
1. Place robot with front bumper at a known distance from hub base
2. Use tape measure — measure from **front bumper** to **hub base**
3. Verify `Vision/correctedBumperDist` matches your tape measure (from Test 5)

### For each distance point:

1. **Position**: Move robot to target distance (start at 1.5m, work outward)
2. **Lock on**: Turret should auto-track the hub tag (state = TRACKING)
3. **Enter calibration mode**: Hold **driver B** (CalibrationShootCommand)
4. **Turret holds center**, hood and shooter controlled via NetworkTables
5. In Elastic, adjust:
   - `CalibrationShoot/hoodAngle` — start at the existing table value for this distance
   - `CalibrationShoot/shooterSpeed` — start at the existing table value
6. **Feed balls** — observe trajectory
7. **Tune hood first**: Adjust hood angle until arc height is correct
   - Too flat / hitting the rim → increase hood angle
   - Too high / sailing over → decrease hood angle
8. **Tune shooter second**: Adjust speed until balls reach the target
   - Falling short → increase speed
   - Overshooting → decrease speed
9. **Record**: (tape_distance_m, hood_rot, shooter_RPS)
10. **Shoot 3-5 balls** at the final values to confirm consistency

### Recommended distance points

| Priority | Distance | Why |
|---|---|---|
| 1 | 2.0m (79") | Most common close shot |
| 2 | 3.5m (138") | Mid-range |
| 3 | 5.0m (197") | Long range |
| 4 | 1.5m (59") | Very close (under the hub) |
| 5 | 4.0m (157") | Fill in mid-range |
| 6 | 6.0m (236") | Long range |
| 7 | 7.0m (276") | Maximum range |

**Minimum viable**: 4-5 points. The interpolation fills in the gaps.

### Update the code

After collecting all data points, update `TurretAimingCalculator.java`:

```java
// ── Flywheel speed table (distance m → RPS) ──
flywheelRPSTable = new InterpolatingDoubleTreeMap();
flywheelRPSTable.put(DIST_1, RPS_1);
flywheelRPSTable.put(DIST_2, RPS_2);
// ... etc

// ── Hood angle table (distance m → mechanism rotations) ──
hoodAngleTable = new InterpolatingDoubleTreeMap();
hoodAngleTable.put(DIST_1, HOOD_1);
hoodAngleTable.put(DIST_2, HOOD_2);
// ... etc
```

Deploy and re-test at a couple of distances to verify.

---

## Telemetry for debugging shots

While shooting, watch:
- `TurretAutoAimCommand/distanceToHubMeters` — does it match your tape measure?
- `Shooter/shooterSetpointRPS` — is the interp table outputting the right speed?
- `Shooter/shooterVelocityRPS` — has the shooter reached setpoint before feeding?
- `Hood/hoodPositionRot` — is the hood at the right angle?
- `TurretAutoAimCommand/visionTxDeg` — is the turret aimed? (should be < 1°)

### If balls go left/right (not straight)
- Turret aim is off → revisit Test 6, check `TURRET_AIM_OFFSET`
- Or ball is contacting the hood asymmetrically → mechanical issue

### If balls are consistently short at all distances
- Shooter speed too low → increase all RPS values by 5-10%
- Or battery is low → check battery voltage during shooting

### If balls are good at close range but off at far range
- The interp table is non-linear and you need more data points in the transition zone
- Add a point between where it's good and where it's bad

---

## Quick validation after deployment

Pick 3 distances you calibrated at. At each:
1. Lock turret on tag
2. Hold right trigger to shoot
3. Verify 3/5 balls score

If 3/5 don't score, the distance read or the table values are off for that range.

---

## Pass criteria
- [ ] At least 5 distance points calibrated
- [ ] Table updated in TurretAimingCalculator.java
- [ ] Code deployed and build passes
- [ ] 3/5 balls score at 2.0m
- [ ] 3/5 balls score at 3.5m
- [ ] 3/5 balls score at 5.0m
- [ ] Distance telemetry matches tape measure at each point
