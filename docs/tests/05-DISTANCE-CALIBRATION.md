# Test 5: Distance Calibration — Camera-to-Bumper Offset

**Goal**: Measure the camera-to-bumper offset so that the distance reported by the turret camera matches a tape measure from bumper to hub. This is the single most important calibration for accurate shooting — if distance is wrong, the interp table outputs wrong hood/shooter values.

**Time**: ~20 minutes

**Prerequisites**: Tests 0-4 passed. Tape measure. AprilTag on a wall or the actual hub.

---

## Placeholder values resolved in this test

| Value | Current | What to set |
|---|---|---|
| `CAMERA_TO_BUMPER_OFFSET_METERS` | 20" (0.508m) | **Measure on robot** |
| `ROBOT_TO_TURRET_PIVOT` | estimated | Verify or re-measure |
| `TURRET_PIVOT_TO_CAM` | estimated | Verify or re-measure |

---

## Background: How distance works

```
         Camera ──────── horizontal_dist ────────── Tag
         │                                          │
         │◄── CAMERA_TO_BUMPER_OFFSET ──►│          │
         │                               │          │
      [camera]                      [front bumper]  [hub]
         │                               │          │
         └────── correctedBumperDist ──────────────►│
```

The interp tables were calibrated by measuring **bumper-to-hub** with a tape measure. But the camera is NOT at the bumper — it's mounted somewhere behind it on the turret. The offset bridges this gap.

`correctedBumperDist = horizontal_camera_dist + CAMERA_TO_BUMPER_OFFSET_METERS`

---

## Telemetry to watch

- `Vision/horizontalCameraDist` — camera-to-tag, horizontal only (Z stripped)
- `Vision/correctedBumperDist` — after adding bumper offset (this feeds interp tables)
- `Vision/rawCameraDist3d` — raw 3D distance (includes height difference)
- `Vision/cameraToTargetX` — forward component of camera-to-tag vector
- `Vision/cameraToTargetY` — lateral component
- `Vision/cameraToTargetZ` — vertical component (should be non-zero due to height diff)

---

## Step 1: Physical measurement of camera-to-bumper

1. Aim the turret straight forward (D-pad Up = 0.0 position)
2. With the turret aimed forward, measure from the **camera lens** to the **front bumper face** along the turret's forward axis
3. Use a tape measure or ruler. Measure in inches, convert to meters.
4. Record: `camera_to_bumper = ___ inches = ___ meters`

**Example**: If the camera lens is 18" behind the front bumper → offset = 18" = 0.457m

## Step 2: Set the offset

Update `VisionConstants.java` line 174:
```java
public static final double CAMERA_TO_BUMPER_OFFSET_METERS = Units.inchesToMeters(<YOUR_INCHES>);
```

Deploy the code.

## Step 3: Validate at a known distance

1. Place the robot so the **front bumper** is exactly **2 meters** (78.7") from an AprilTag
2. Use a tape measure — be precise
3. Aim turret at the tag (state = TRACKING)
4. Read `Vision/correctedBumperDist` in Elastic

### Expected result
`correctedBumperDist` should read ~2.0 meters (within ±0.1m).

### If it reads too high (e.g. 2.3m at a 2.0m measured distance)
- `CAMERA_TO_BUMPER_OFFSET_METERS` is too large
- Reduce the offset by the difference (0.3m = ~12")
- Re-deploy and re-check

### If it reads too low (e.g. 1.7m at a 2.0m measured distance)
- Offset is too small
- Increase by the difference (0.3m)

## Step 4: Validate at multiple distances

Repeat the tape-measure check at 3 distances:

| Tape measure (bumper to tag) | `correctedBumperDist` | Error |
|---|---|---|
| 1.5m (59") | ___ m | ___ m |
| 3.0m (118") | ___ m | ___ m |
| 5.0m (197") | ___ m | ___ m |

The error should be **consistent** across all distances (±0.1m). If the error grows with distance, the camera horizontal distance calculation itself may be off — check the camera transform (pitch angle especially).

## Step 5: Verify camera transform components

While aimed at a tag at ~3m:
- `cameraToTargetX` should be large and positive (~2.5-3.0m) — forward distance
- `cameraToTargetY` should be small (< 0.3m) — lateral offset, near zero when aimed
- `cameraToTargetZ` should be negative (camera is higher than tag base) or positive (tag is higher)

If `cameraToTargetX` is negative or near zero, the camera transform may be flipped. Check `ROBOT_TO_TURRET_PIVOT` and `TURRET_PIVOT_TO_CAM` in VisionConstants.

---

## Debugging: Why distance might be wrong

| Symptom | Likely cause |
|---|---|
| Distance is always ~0.5m off by the same amount | Offset needs adjustment (most common) |
| Distance error grows with range | Camera pitch angle wrong in transform |
| Distance jumps between frames | Tag ambiguity — multiple solutions. Check `MAX_AMBIGUITY`. |
| `horizontalCameraDist` = 0.0 | Camera doesn't see a tag or result is stale |
| `rawCameraDist3d` ≫ `horizontalCameraDist` | Large height difference — normal for tags mounted high |

---

## Pass criteria
- [ ] `CAMERA_TO_BUMPER_OFFSET_METERS` physically measured and set
- [ ] `correctedBumperDist` matches tape measure within ±0.1m at 1.5m
- [ ] `correctedBumperDist` matches tape measure within ±0.1m at 3.0m
- [ ] `correctedBumperDist` matches tape measure within ±0.1m at 5.0m
- [ ] Error is consistent across distances (not growing)
