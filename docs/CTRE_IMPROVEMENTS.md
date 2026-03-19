# CTRE Phoenix 6 — Missed Opportunities & Improvements

Findings from ingesting the official CTRE Phoenix 6 documentation against our current codebase.

---

## What We're Already Doing Right

- **Voltage-based control** (MotionMagicVoltage, VelocityVoltage) instead of DutyCycle — more stable and reproducible behavior since output isn't affected by battery voltage fluctuation.
- **`optimizeBusUtilization()`** on all devices — reduces CAN bus traffic by disabling unused status signals.
- **Custom update frequencies** — only polling signals we actually need, at appropriate rates (50Hz for feedback, 4-10Hz for telemetry).
- **Retry loops on config apply** — all subsystems retry `getConfigurator().apply()` up to 5 times, which handles CAN bus initialization race conditions.
- **Stator + supply current limits** on every motor — protects motors, battery, and breakers.
- **Software soft limits** on turret, hood, intake — prevents mechanical damage.
- **MotionMagic jerk limiting** — S-curve profiles reduce oscillation on turret, hood, intake.
- **Peak voltage limits (10V)** — now applied across all motors via config.

---

## Opportunities We're NOT Capitalizing On

### 1. ~~Intake Pivot: Missing Gravity Compensation~~ — NOT NEEDED

**Status**: Evaluated and ruled out. The intake deploys all the way out until it hits the bumpers as a hard stop. It doesn't need to hold a mid-travel position against gravity. When stowed, internal friction holds it. `kG = 0.0` is correct for this mechanism.

---

### 2. Motion Magic Expo (MEDIUM PRIORITY)

**What it is**: An exponential motion profile that better matches real motor/mechanism dynamics. Reduces both overshoot AND time-to-target compared to trapezoidal profiles.

**Where to use**: Turret and hood — both are position-controlled mechanisms where faster settling time directly improves shooting accuracy.

**How it works**: Instead of CruiseVelocity + Acceleration + Jerk, you specify:
- `MotionMagicExpo_kV` — volts per RPS to maintain a velocity
- `MotionMagicExpo_kA` — volts per RPS² to apply an acceleration
- Optional cruise velocity cap

The profile naturally accelerates fast when far from target and decelerates smoothly as it approaches — like real physics, not a trapezoidal approximation.

**Implementation**: Replace `MotionMagicVoltage` with `MotionMagicExpoVoltage` in the subsystem code. Requires measuring kV and kA for each mechanism.

```java
// Example for turret
var request = new MotionMagicExpoVoltage(0);
turretMotor.setControl(request.withPosition(target));

// Config
TURRET_CONFIG.MotionMagic.MotionMagicExpo_kV = 0.12;  // V/rps — measure
TURRET_CONFIG.MotionMagic.MotionMagicExpo_kA = 0.01;  // V/(rps/s) — measure
```

**Risk**: Low — if kV and kA are too high, the profile is just slower (safe). Start high, tune down.

---

### 3. Field Oriented Control / FOC (LOW PRIORITY — requires Pro license)

**What it is**: A commutation mode that increases peak power by ~15% on all motors.

**How to enable**: Add `.withEnableFOC(true)` to control requests.

```java
shooterMotionRequest = new VelocityVoltage(0).withEnableFOC(true);
turretMotionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
```

**Requirement**: Each TalonFX must be Pro licensed (via season license or CANivore). Check if your motors have Pro licenses before enabling. Without a license, the motor will fault and disable output.

**Benefit**: 15% more power from the same motor. Most useful for the shooter flywheel (faster spin-up, more consistent shot energy).

---

### 4. Two-Tier Supply Current Limiting (MEDIUM PRIORITY)

**What it is**: Phoenix 6 supports a `SupplyCurrentLowerTime` and `SupplyCurrentLowerLimit`. If supply current exceeds the main limit for longer than the LowerTime, it drops to the LowerLimit until current falls below.

**Why it matters**: Prevents breaker trips during sustained loads without limiting peak performance. Currently we only have a hard cap.

**Where to use**: Shooter (sustained high current during rapid-fire sequences) and drivetrain (sustained pushing).

```java
// Example for shooter
SHOOTER_CONFIG.CurrentLimits.SupplyCurrentLimit = 60.0;           // allow 60A peaks
SHOOTER_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 40.0;      // after sustained load, drop to 40A
SHOOTER_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 1.0;        // trigger after 1 second at 60A
```

---

### 5. Turret Continuous Wrap (EVALUATE — may not apply)

**What it is**: `ContinuousWrap` tells the motor controller to take the shortest path to a target, treating the mechanism as having unlimited rotation.

**Why it might not apply**: Our turret has soft limits (-0.58 to +0.20 rot) due to cable management. ContinuousWrap assumes the mechanism can rotate freely past 1.0 rotation. If the turret can't physically wrap around, this feature would cause it to try and hit the hard stops.

**Verdict**: Do NOT enable unless the turret gets a slip ring or cable-safe continuous rotation.

---

### 6. ~~Turret kS (Static Friction Feedforward)~~ — NOT WORTH IT

**Status**: Evaluated and ruled out. The turret's cable routing creates uneven, position-dependent friction. A constant kS would be correct at one turret angle but wrong at others. The kI gain (5.0) already handles pushing through friction at any position. Leave kS = 0.0.

---

## Summary — Priority Order

| # | Improvement | Effort | Impact | Risk |
|---|---|---|---|---|
| 1 | ~~Intake kG~~ | N/A | N/A | **Not needed** — hard stop at bumpers |
| 2 | Two-tier supply current | Low (config change) | Medium — prevents breaker trips | None |
| 3 | MotionMagic Expo | Medium (code + measurement) | Medium — faster settling | Low |
| 4 | FOC (EnableFOC) | Trivial (one line per request) | Medium — 15% more power | Requires Pro license |
| 5 | Continuous Wrap | N/A | N/A | **Do not enable** (soft limits) |
| 6 | ~~Turret/Hood kS~~ | N/A | N/A | **Not needed** — cable friction is variable |
