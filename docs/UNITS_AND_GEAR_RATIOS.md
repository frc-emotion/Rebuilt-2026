# Units, Gear Ratios, and Conversions — Rebuilt 2026

This document is the single reference for every unit convention, gear ratio, and conversion factor used in the robot code.

---

## Universal Unit Conventions (CTRE Phoenix 6)

All CTRE TalonFX motors report and accept values in **mechanism units** after applying gear ratios:

| Quantity | Unit | Example |
|---|---|---|
| Position | **Rotations** (of the mechanism output) | Turret: 0.20 rot = 72 degrees |
| Velocity | **RPS** (rotations per second of mechanism) | Shooter: 60 RPS |
| Acceleration | **RPS²** | MotionMagic accel: 4.0 RPS² |
| Voltage | **Volts** | PeakForwardVoltage: 10.0V |
| Current | **Amps** | StatorCurrentLimit: 80A |

**Key insight**: When `SensorToMechanismRatio` is set, the TalonFX firmware automatically converts between rotor ticks and mechanism output. You never deal with raw encoder ticks in this codebase.

---

## WPILib Unit Conventions

| Quantity | Unit | Notes |
|---|---|---|
| Distance | **Meters** | All field measurements, transforms, distances |
| Angle | **Radians** (internal), but `Rotation2d` wraps this | `Rotation2d.fromRotations(0.5)` = 180 degrees |
| Translation | **(X, Y, Z) in meters** | X = forward, Y = left, Z = up |
| Rotation | **(Roll, Pitch, Yaw) in radians** | Yaw positive = CCW |
| Time | **Seconds** | FPGA timestamp, Timer |

**WPILib coordinate system:**
```
      +X (forward/front of robot)
       ^
       |
+Y <---+---> -Y
(left)  |   (right)
       v
      -X (backward)
       
+Z = up from floor
Yaw: positive = counter-clockwise (from above)
Pitch: negative = tilted up from horizontal
```

---

## Gear Ratios — Complete Reference

### Turret
```
Motor Rotor ──[5.08:1]──> Turret Output

Config: SensorToMechanismRatio = 5.08
Sensor: RotorSensor (internal TalonFX encoder)

Meaning: 5.08 motor turns = 1 turret rotation = 360 degrees
         1 motor turn = 70.87 degrees of turret rotation
         0.01 turret rotations = 3.6 degrees
```

### Hood
```
Motor Rotor ──[12.917:1]──> Hood Output

Config: SensorToMechanismRatio = 155.0 / 12.0 = 12.917
Sensor: RotorSensor (internal TalonFX encoder)

Meaning: 12.917 motor turns = 1 hood rotation = 360 degrees
         Total hood travel: 0.08 rotations = 28.8 degrees
         0.001 hood rotations = 0.36 degrees
```

### Shooter (Flywheel)
```
Motor Rotor ──[1:1]──> Flywheel

Config: SensorToMechanismRatio = 1.0 (default, not explicitly set)
Sensor: RotorSensor

Meaning: Motor RPS = Flywheel RPS
         60 RPS = 3600 RPM
```

### Intake Pivot
```
Motor Rotor ──[27:1]──> CANcoder ──[1:1]──> Pivot Output

Config: RotorToSensorRatio = 27.0
        SensorToMechanismRatio = 1.0
Sensor: RemoteCANcoder (ID 22)

Meaning: 27 motor turns = 1 CANcoder turn = 1 pivot turn = 360 degrees
         Pivot travel: 0.3 to 0.653 rot = 0.353 rot = 127 degrees
```

### Intake Roller
```
Motor Rotor ──[1:1]──> Roller

No gear ratio set. Motor RPS = Roller RPS.
Operating speed: 55 RPS
```

### Indexers (all three)
```
Motor Rotor ──[1:1]──> Indexer wheels

No gear ratio set. Motor RPS = Wheel RPS.
Operating speed: 50 RPS
```

### Climb
```
Motor Rotor ──[?:1]──> CANcoder ──[1:1]──> Climb Output

Config: RotorToSensorRatio = 1.0 (NOTE: potentially wrong — GEAR_RATIO = 12.0 exists but unused)
        SensorToMechanismRatio = 1.0
Sensor: RemoteCANcoder (ID 67)

Meaning: Currently 1:1 motor-to-output. The GEAR_RATIO = 12.0 constant suggests
         the actual mechanical ratio is 12:1 but hasn't been applied in firmware.
         TODO: Verify and fix if needed.
```

---

## Position Ranges — Quick Reference

| Mechanism | Min | Max | Total Travel | Degrees |
|---|---|---|---|---|
| Turret | -0.58 rot | +0.20 rot | 0.78 rot | 280.8° |
| Hood | 0.00 rot | 0.08 rot | 0.08 rot | 28.8° |
| Intake pivot | 0.30 rot (stowed) | 0.653 rot (deployed) | 0.353 rot | 127° |
| Climb | 0.0 rot | 1000 rot (max) | 1000 rot | N/A (linear) |

---

## Interpolation Table Distance Units

The `TurretAimingCalculator` interpolation tables are keyed by **distance in meters**:

```
Key (meters) → Value (mechanism rotations for hood, or RPS for shooter)

Example: distance = 3.232 m → hood = 0.031 rot, shooter = 55 RPS
```

The distance fed into these tables comes from `getTurretCameraDistanceToTarget()`:
```
camera_horizontal_dist (meters) + CAMERA_TO_BUMPER_OFFSET_METERS (currently 0.508m / 20")
```

The tables were calibrated by measuring **bumper-to-hub distance with a tape measure**, so the camera distance must be converted to match.

---

## Conversion Quick Reference

| From | To | Formula |
|---|---|---|
| Rotations → Degrees | × 360 | 0.20 rot = 72° |
| Degrees → Rotations | ÷ 360 | 90° = 0.25 rot |
| Inches → Meters | × 0.0254 | 20" = 0.508 m |
| Meters → Inches | × 39.37 | 1.0 m = 39.37" |
| RPS → RPM | × 60 | 50 RPS = 3000 RPM |
| Degrees/sec → RPS | ÷ 360 | 360°/s = 1.0 RPS |

---

## CAN IDs — Complete Map

| ID | Device | Bus |
|---|---|---|
| 20 | Intake pivot motor | mechanisms |
| 21 | Intake roller motor | mechanisms |
| 22 | Intake pivot CANcoder | mechanisms |
| 31 | Horizontal indexer motor | mechanisms |
| 32 | Vertical indexer motor | mechanisms |
| 33 | Upward indexer motor | mechanisms |
| 40 | Climb leader motor | mechanisms |
| 41 | Climb follower motor | mechanisms |
| 50 | Shooter motor | mechanisms |
| 51 | Turret motor | mechanisms |
| 52 | Hood motor | mechanisms |
| 53 | Turret CANcoder | mechanisms |
| 54 | Hood CANcoder | mechanisms |
| 67 | Climb CANcoder | mechanisms |
| (Pigeon2) | IMU | rio (default) bus |
| (Swerve) | 4 drive + 4 steer + 4 CANcoder | rio (default) bus |
