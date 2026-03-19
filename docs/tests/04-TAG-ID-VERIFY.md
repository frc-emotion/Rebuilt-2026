# Test 4: Verify DS-Facing Hub Tag IDs

**Goal**: Confirm which AprilTag IDs on the physical hub face toward the driver station for each alliance. Wrong IDs = turret locks onto far-side tags and gets bad aim data.

**Time**: ~15 minutes

**Prerequisites**: Test 0, 3 passed. Access to the actual field or a field map with tag IDs.

---

## Placeholder values resolved in this test

| Value | Current | What to set |
|---|---|---|
| `RED_DS_FACING_TAG_IDS` | {9, 10} | Verified IDs from field |
| `BLUE_DS_FACING_TAG_IDS` | {25, 26} | Verified IDs from field |

---

## Why this matters

The hub has ~8 AprilTags arranged around it. Only 2-3 face toward your driver station. If the turret locks onto a tag on the **far side** of the hub, the `tx` (yaw error) will be wildly wrong — it'll aim at the back of the hub instead of the front.

## Method 1: Check on the physical field

1. Stand at the **red driver station** looking toward the red hub
2. Walk up to the hub and identify which tag IDs are **visible from** the driver station side
3. Record those IDs → these are `RED_DS_FACING_TAG_IDS`
4. Repeat from the **blue driver station** → `BLUE_DS_FACING_TAG_IDS`

## Method 2: Check from the field layout JSON

1. Open `src/main/deploy/FRC2026_WELDED.json`
2. For each hub tag, look at the quaternion rotation
3. Tags facing the **red DS** (at x ≈ 16.5m) will have a rotation that points in the +X direction
4. Tags facing the **blue DS** (at x ≈ 0m) will have a rotation that points in the -X direction

Our current analysis from the JSON suggests:
- **Red DS-facing**: tags 9, 10 (face +X)
- **Blue DS-facing**: tags 25, 26 (face -X)

## Method 3: Drive the robot and check telemetry

1. Set `BENCH_TEST_ANY_TAG = true` temporarily
2. Drive the robot to the **red** scoring zone, facing the hub
3. Watch `TurretAutoAimCommand/trackedTagId` in Elastic
4. Note which tag IDs appear as the turret tracks
5. The tag that gives the **most consistent tx** (smallest, most centered) from a typical scoring position is the correct DS-facing tag
6. Repeat from the blue side

## Verification

After determining the correct IDs:

1. Update `VisionConstants.java`:
   ```java
   public static final int[] RED_DS_FACING_TAG_IDS = { <your_ids> };
   public static final int[] BLUE_DS_FACING_TAG_IDS = { <your_ids> };
   ```
2. Set `BENCH_TEST_ANY_TAG = false`
3. Deploy and test:
   - From red scoring zone: `trackedTagId` should ONLY show red DS-facing IDs
   - From blue scoring zone: `trackedTagId` should ONLY show blue DS-facing IDs
   - From the far side of the hub: state should stay **MANUAL** (no DS-facing tags visible)

---

## Pass criteria
- [ ] Red DS-facing tag IDs confirmed and updated
- [ ] Blue DS-facing tag IDs confirmed and updated
- [ ] Turret only tracks DS-facing tags from scoring position
- [ ] Turret does NOT track far-side tags
- [ ] `BENCH_TEST_ANY_TAG` set back to `false`
