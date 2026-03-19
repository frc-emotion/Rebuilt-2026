# Test 3: Camera Basics — Connectivity, Tags, Latency

**Goal**: Confirm the turret camera detects AprilTags and that pipeline latency is acceptable for tracking.

**Time**: ~10 minutes

**Prerequisites**: Test 0 passed. At least one AprilTag visible to the robot.

---

## Telemetry to watch

- `Vision/visionLatencyMs` — pipeline processing time (goal: <50ms)
- `Vision/timeSinceLastFrameMs` — total time since last fresh frame (goal: <100ms)
- `TurretAutoAimCommand/trackedTagId` — which tag is being tracked (-1 = none)
- `TurretAutoAimCommand/visionTxDeg` — yaw error to tag center
- `TurretAutoAimCommand/state` — "MANUAL" or "TRACKING"

---

## Step 1: Verify camera connection in code

1. Enable robot in teleop
2. Point turret camera at an AprilTag (any ID for now)
3. Set `BENCH_TEST_ANY_TAG = true` in `VisionConstants.java` line 48 if you don't have hub tags
4. Deploy and re-enable

## Step 2: Check state transition

1. With tag visible: `state` should be **"TRACKING"**
2. `trackedTagId` should show the tag's ID number (not -1)
3. `visionTxDeg` should show a value (how far off-center the tag is)
4. Block the camera with your hand — state should switch to **"MANUAL"** after ~0.15s
5. Unblock — state should return to **"TRACKING"** instantly

### If state stays MANUAL even with a visible tag

**Debug checklist** (in order):
1. Check `ENABLE_TURRET_CAM` = true in VisionConstants
2. Check `TURRET_CAM_NAME` matches the camera name in PhotonVision web UI exactly
3. Open PhotonVision UI — does it show a green bounding box on the tag?
4. Check `BENCH_TEST_ANY_TAG` — if false, only DS-facing hub tags trigger tracking
5. Check console for vision errors

## Step 3: Measure latency

With tag visible and state = TRACKING:

1. Read `Vision/visionLatencyMs` — this is the PhotonVision pipeline time
   - **Good**: < 30ms
   - **Acceptable**: 30-50ms
   - **Problem**: > 50ms — reduce camera resolution or simplify pipeline
   
2. Read `Vision/timeSinceLastFrameMs` — this is total end-to-end latency
   - **Good**: < 60ms
   - **Acceptable**: 60-100ms
   - **Problem**: > 100ms — check USB bandwidth, coprocessor CPU usage

### If latency is too high

- In PhotonVision UI: reduce camera resolution (640x480 is enough)
- In PhotonVision UI: reduce exposure time
- Check coprocessor CPU usage — if >80%, the coprocessor is overloaded
- Make sure only ONE pipeline is active per camera

## Step 4: Verify tx range

1. Slowly move the tag left and right in front of the camera
2. `visionTxDeg` should range from roughly -25° to +25° (depends on camera FOV)
3. Positive tx = tag is to the right of camera center
4. Negative tx = tag is to the left

The sign convention matters for tracking — if the turret moves **away** from the tag when tracking, the sign is inverted. This will be caught in Test 6.

---

## Pass criteria
- [ ] State switches to TRACKING when tag is visible
- [ ] State switches to MANUAL when tag is blocked
- [ ] `trackedTagId` shows correct tag ID
- [ ] `visionLatencyMs` < 50ms
- [ ] `timeSinceLastFrameMs` < 100ms
- [ ] `visionTxDeg` updates as tag moves left/right
