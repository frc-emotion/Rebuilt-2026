# PID and Control Analysis — Rebuilt 2026

Deep analysis of every PID loop on the robot: what each gain does, why the current values are what they are, what ideal values look like, and how to tune them.

---

## How CTRE Phoenix 6 Control Loops Work

CTRE Phoenix 6 runs PID on the **motor controller** (not the roboRIO) at 1 kHz. The control output equation depends on the control mode:

### MotionMagicVoltage (Position control with motion profiling)
```
output_volts = kG + kS×sign(vel) + kV×profile_velocity + kA×profile_accel + kP×error + kI×∫error + kD×d(error)/dt
```
The motion profile generator creates a trapezoidal velocity profile, and the PID corrects any deviation from that profile. This is used by: **turret, hood, intake pivot, climb**.

### VelocityVoltage (Velocity control)
```
output_volts = kS×sign(target) + kV×target_velocity + kA×target_accel + kP×velocity_error + kI×∫vel_error + kD×d(vel_error)/dt
```
The feedforward (kS + kV) provides most of the output; PID corrects the residual. Used by: **shooter, indexers, intake roller**.

---

## Subsystem-by-Subsystem Analysis

### 1. Turret — MotionMagicVoltage (Position)

**Current values:**
| Gain | Value | Status |
|---|---|---|
| kP | 20.0 | Reasonable starting point |
| kI | 5.0 | Active — eliminates steady-state error |
| kD | 0.0 | Not set |
| kS | 0.0 | Not set |
| kV | 0.0 | Not set |
| kG | 0.0 | Correct — no gravity on horizontal rotation |

**Analysis:**

The turret gear ratio is 5.08:1 and the mechanism has low inertia (just the turret plate + camera + hood + shooter). The total range is 0.78 rotations.

- **kP = 20.0**: At the worst-case error of 0.78 rot (full range), this produces 15.6V — more than the motor can accept (capped at 12V by battery). But in practice, MotionMagic never generates that much error because the profile limits velocity/acceleration. Typical tracking errors are 0.01-0.05 rot, producing 0.2-1.0V corrections. **This is appropriate.**

- **kI = 5.0**: This is relatively high. At a sustained 0.01 rot error for 1 second, kI contributes 0.05V — just enough to push through cable drag. However, high kI can cause windup if the turret hits a soft limit. **Consider adding kI winddown or reducing to 2.0-3.0 if oscillation occurs at limits.**

- **kD = 0.0**: MotionMagic's profile already handles deceleration, so kD is less critical. However, adding kD = 1.0-5.0 could help dampen overshoot during fast tracking changes. **Optional improvement.**

- **kS = 0.0**: **Intentionally zero.** The turret's cable routing creates uneven, unpredictable friction that varies with turret position. A constant kS would be correct at one angle but wrong at others. The kI term handles pushing through friction instead.

**Ideal values (estimated):**
| Gain | Suggested | Reason |
|---|---|---|
| kP | 15-25 | Current 20 is good |
| kI | 2.0-5.0 | Reduce if oscillation at limits |
| kD | 0.0-3.0 | Optional dampening |
| kS | 0.0 | Leave at zero — cable friction is position-dependent |

**MotionMagic profile:**
| Parameter | Current | Analysis |
|---|---|---|
| CruiseVelocity | 1.0 RPS | 360°/s — fast. Could reduce to 0.5 if turret overshoots. |
| Acceleration | 4.0 RPS² | Reaches cruise in 0.25s. Reasonable. |
| Jerk | 40.0 | S-curve smoothing. Good for preventing jerks. |

---

### 2. Hood — MotionMagicVoltage (Position)

**Current values:**
| Gain | Value | Status |
|---|---|---|
| kP | 100.0 | Very high — justified by tiny range |
| kI | 50.0 | Very high — may cause issues |
| kD | 0.0 | Not set |
| kS | 0.0 | Leave at zero — gear friction is high enough |
| kG | 0.0 | **Intentionally zero** — hood does not move under gravity |

**Analysis:**

The hood gear ratio is 12.917:1 and total travel is only 0.08 rotations. The mechanism is small and high-friction (tight gears).

- **kP = 100.0**: Looks extreme, but consider the range. A "large" error of 0.01 rot is 12.5% of total travel and produces 1.0V. A typical operating error of 0.001 rot (0.36°) produces 0.1V. **This is appropriate for the tiny range.** Equivalent to kP=10 on a mechanism with 0.8 rot range.

- **kI = 50.0**: At 0.001 rot sustained error for 1 second, contributes 0.05V. Given the high gear friction, this helps reach exact position. However, if the hood oscillates around setpoint, this is the first thing to reduce. **Reduce to 20-30 if oscillation occurs.**

- **kG = 0.0**: **Intentionally zero.** The hood's gear friction is high enough that it does not drift under gravity when unpowered. The MotionMagic default command holds position actively anyway, so kG is unnecessary.

**Ideal values (estimated):**
| Gain | Suggested | Reason |
|---|---|---|
| kP | 80-120 | Current 100 is good |
| kI | 20-50 | Reduce if oscillation |
| kD | 0.0-5.0 | Optional |
| kS | 0.0 | Leave at zero — gear friction is high enough |
| kG | 0.0 | Leave at zero — hood doesn't drift under gravity |

---

### 3. Shooter — VelocityVoltage

**Current values:**
| Gain | Value | Status |
|---|---|---|
| kS | 0.15 | Good starting point |
| kV | 0.12 | PRIMARY GAIN — provides most of output |
| kP | 0.3 | Fine correction |
| kI | 0.0 | Not needed for flywheel |
| kD | 0.0 | Not needed for flywheel |

**Analysis:**

The shooter is a direct-drive flywheel (1:1 ratio). It has high inertia (heavy wheel) which acts as natural dampening.

- **kV = 0.12**: This is the most important gain. At 60 RPS, feedforward = 0.15 + 0.12×60 = 7.35V. At 74 RPS (max from table), feedforward = 0.15 + 0.12×74 = 9.03V. Both under the 10V cap. **This seems well-tuned.**

- **kS = 0.15**: Static friction compensation. The motor needs 0.15V just to overcome bearing friction before it starts accelerating. **Typical for a Kraken.**

- **kP = 0.3**: At 1 RPS error, adds 0.3V correction. At 5 RPS error, adds 1.5V. Helps with spin-up response and maintaining speed after a ball steals energy. **Good value.**

- **Voltage limits**: PeakForwardVoltage = 10V, PeakReverseVoltage = 0V. The 10V cap limits spin-up torque slightly but protects the battery. The 0V reverse means the flywheel always coasts down — it never actively brakes.

**Ideal values:**
| Gain | Suggested | Reason |
|---|---|---|
| kS | 0.10-0.20 | Measure: minimum voltage to start spinning |
| kV | 0.10-0.14 | Measure: voltage at steady-state speed ÷ speed |
| kP | 0.2-0.5 | Increase for faster recovery after ball impact |
| kI | 0.0 | Not needed — no static load on flywheel |

**How to measure kV precisely:**
1. Command a known speed (e.g., 60 RPS)
2. Wait for steady state
3. Read `Shooter/shooterVoltageVolts` from telemetry
4. `kV = (voltage - kS) / speed`
5. Example: (7.5V - 0.15V) / 60 = 0.1225 → round to 0.12 ✓

---

### 4. Indexers — VelocityVoltage (all three identical)

**Current values (all three):**
| Gain | Value |
|---|---|
| kS | 0.15 |
| kV | 0.12 |
| kP | 0.3 |

**Analysis:**
Indexers are simple velocity-controlled motors pushing balls. They don't need precise speed — just "fast enough." The gains are copied from the shooter and work fine. No tuning needed unless balls get stuck or indexers stall.

---

### 5. Intake Pivot — MotionMagicVoltage (Position)

**Current values:**
| Gain | Value | Status |
|---|---|---|
| kP | 13.0 | Moderate |
| kI | 0.0 | Not set |
| kD | 0.0 | Not set |
| kS | 0.0 | Not set |
| kG | 0.0 | **SHOULD BE SET** — arm has gravity load |

**Analysis:**

The intake pivot has a 27:1 gear reduction and 0.353 rot of travel. It's an arm that extends outside the frame — gravity pulls it down when deployed.

- **kP = 13.0**: At 0.05 rot error (~18°), produces 0.65V. Moderate — could be higher for faster response, but the MotionMagic profile limits the trajectory anyway. **Acceptable.**

- **kG = 0.0**: **This is the biggest issue.** The intake arm has significant gravity torque, especially when extended. Without kG, the PID must generate all the holding force via error accumulation (which requires kI) or accepting a constant position error. **Setting kG = 0.3-0.8V would dramatically improve position holding.**

- **kI = 0.0**: Without kG, there's no integral to fight gravity. The arm will sag below its setpoint by an amount proportional to the gravity torque divided by kP. **Either add kI = 2-5 or (better) add kG.**

**Ideal values (estimated):**
| Gain | Suggested | Reason |
|---|---|---|
| kP | 10-20 | Current 13 is fine |
| kI | 0.0-3.0 | Add if kG alone doesn't eliminate error |
| kD | 0.0-2.0 | Optional dampening |
| kS | 0.05-0.2 | Measure |
| kG | 0.3-0.8 | **Critical** — measure by finding voltage to hold arm level |

**How to measure kG:**
1. Deploy intake to horizontal position
2. Slowly reduce voltage until arm just barely holds position
3. That voltage is kG
4. Set `INTAKE_CONFIG.Slot0.GravityType = GravityTypeValue.Arm_Cosine` (gravity varies with angle)

---

### 6. Intake Roller — VelocityVoltage

**Current values:**
| Gain | Value |
|---|---|
| kS | 0.15 |
| kV | 0.12 |
| kP | 0.3 |

Same as indexers/shooter. Works fine for a roller that just needs to spin.

---

### 7. Climb — MotionMagicVoltage (Position)

**Current values:**
| Gain | Value | Status |
|---|---|---|
| kP | 0.1 | Very low |
| kG | 0.5 | Elevator gravity compensation |
| kI | 0.0 | Not set |

**Analysis:**

The climb is an elevator-type mechanism that lifts the robot's weight. `GravityType = Elevator_Static` means kG provides a constant upward force regardless of position (unlike arm which varies with angle).

- **kP = 0.1**: Very conservative. The climb doesn't need to be fast — safety matters more. **Appropriate for initial testing.**
- **kG = 0.5**: Provides 0.5V constant hold force against gravity. This needs to be measured — if the robot slides down, increase kG. **Measure on real robot.**

**Note**: MotionMagic constraints are commented out. The climb will use default (slow) motion profiles. Uncomment and tune when ready.

**Status**: Climb is disabled (`climb = null`). Not ready for testing.

---

## Live Tuning with SuperstructureTuner

The `SuperstructureTuner` class allows changing PID gains via Elastic dashboard without redeploying code. It's called every cycle from `Robot.robotPeriodic()`.

### How to use:
1. Open Elastic dashboard
2. Navigate to SuperstructureTuner entries (NetworkTables)
3. Change a gain value (e.g., Turret kP from 20 to 15)
4. The tuner detects the change and hot-applies it to the motor controller
5. Effect is immediate — no restart needed

### What can be tuned:
- kP, kI, kD, kS, kV, kG for each Slot 0
- MotionMagic CruiseVelocity, Acceleration, Jerk
- For turret, hood, shooter, and intake

### Safety note:
Changes via SuperstructureTuner are **volatile** — they reset on power cycle. Once you find good values, update the constants in `TurretConstants.java` or `IntakeConstants.java` permanently.

---

## PID Tuning Cheat Sheet

### Position control (MotionMagic)
1. Start with only kP. Set it high enough that the mechanism reaches setpoint.
2. If it oscillates → reduce kP or add kD.
3. If it doesn't quite reach setpoint → add small kI (start at kP/10).
4. If it jerks at start of movement → reduce MotionMagic Acceleration or add Jerk smoothing.
5. If it fights gravity → add kG.
6. Measure kS: the voltage where the mechanism just barely starts moving.

### Velocity control (VelocityVoltage)
1. Measure kV first: `kV = steady_state_voltage / steady_state_speed`
2. Measure kS: minimum voltage to start spinning.
3. Set kP for error correction: start at 0.1, increase until responsive but not oscillating.
4. kI is almost never needed for velocity control.
5. kD is almost never needed for high-inertia mechanisms (flywheels).

### Warning signs
| Symptom | Likely cause |
|---|---|
| Oscillation around setpoint | kP too high, or kI windup |
| Never reaches setpoint | kP too low, or no kI and friction |
| Violent jerk at start | Acceleration too high, no jerk limit |
| Overshoots then settles | kD too low, or CruiseVelocity too high |
| Drifts over time | kI too low, or kG missing on gravity-loaded mechanism |
| Motor gets hot doing nothing | kI windup — mechanism is at hard stop but kI keeps pushing |
