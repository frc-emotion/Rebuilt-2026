# Test 0: Hardware Check

**Goal**: Verify every motor, encoder, and camera is online before touching PID or vision.

**Time**: ~10 minutes

**Prerequisites**: Robot powered on, Driver Station connected, Elastic dashboard open.

---

## Step 1: CAN Bus Health

1. Open **Phoenix Tuner X** and connect to the robot
2. All devices on the "mechanisms" bus should appear:
   - TalonFX IDs: 20, 21, 31, 32, 33, 50, 51, 52
   - CANcoder IDs: 22, 53, 54
3. All devices on the "rio" bus should appear:
   - 8x swerve TalonFX + 4x swerve CANcoder + Pigeon2
4. Every device should show **green** (no faults)

### If a device is missing
- Check CAN wiring to that device
- Check that the CAN bus name matches ("mechanisms" vs default rio bus)
- Verify CAN ID matches code (see `TurretConstants`, `IndexerConstants`, `IntakeConstants`)
- Check breaker for that circuit

## Step 2: Console Messages

After robot boot, check the Driver Station console for:
```
[Turret] Config readback:
  FeedbackSource: RotorSensor
  SensorToMech: 5.08
  FwdSoftLimit: enabled=true threshold=0.2
  RevSoftLimit: enabled=true threshold=-0.58
[STARTUP] Turret unified command active (MANUAL/TRACKING)
```

### If you see "Could not apply ... configs"
A motor didn't accept its configuration. Power cycle and retry. If persistent, check CAN wiring.

## Step 3: Camera Check

1. Open PhotonVision web UI: `http://<coprocessor-IP>:5800`
2. Select camera **"mugilanr"** (the turret camera)
3. Confirm:
   - Camera is streaming video
   - AprilTag pipeline is selected
   - If an AprilTag is visible in frame, it shows a green bounding box with tag ID

### If camera doesn't appear
- Check USB cable from camera to coprocessor
- Check that coprocessor is powered (LEDs on)
- Check that `TURRET_CAM_NAME` in `VisionConstants.java` matches the PhotonVision camera name exactly

## Step 4: FaultMonitor

In Elastic dashboard, check `FaultMonitor` — should show no faults for any registered motor.

## Step 5: Telemetry Spot-Check

Enable robot in teleop briefly and verify these telemetry values update (not stuck at 0):
- `Turret/turretPositionRot` — should be ~0.0 if turret is forward
- `Shooter/shooterVelocityRPS` — should be 0 (not NaN)
- `Hood/hoodPositionRot` — should be ~0.0

---

## Pass criteria
- [ ] All CAN devices visible in Phoenix Tuner
- [ ] No "Could not apply configs" errors in console
- [ ] Turret camera streaming in PhotonVision UI
- [ ] FaultMonitor shows no faults
- [ ] Telemetry values are updating (not NaN or stuck)
