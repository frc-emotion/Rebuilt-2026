package frc.robot.runtime;

import frc.robot.runtime.config.MechanismConfig;

/**
 * The hardware boundary for one mechanism, built from a {@link MechanismConfig}. Two
 * implementations exist (real Phoenix 6, physics sim) exactly as the legacy per-subsystem IO triads
 * had — this is the generic seam that replaces all six of them. Logic reads only {@link #state()};
 * it never touches a motor controller directly.
 */
interface MechanismHandle {

  /** Read every sensor value once per loop (real: refreshAll; sim: advance physics). */
  void updateInputs();

  /** The most recent reads (valid after {@link #updateInputs()}). */
  MechanismState state();

  /** Closed-loop velocity, rotations per second; negative reverses. */
  void applyVelocity(double rps);

  /** MotionMagic position (mechanism rotations) with an arbitrary feedforward in volts. */
  void applyPosition(double rot, double feedforwardVolts);

  /** Open-loop voltage (manual jog); firmware soft limits remain active. */
  void applyVoltage(double volts);

  /** Neutral output (coast/brake per config) — the legacy stop() / NeutralOut semantics. */
  void neutral();

  /** Redefine the current physical position as zero (turret boot zero + operator re-zero). */
  void zero();

  MechanismConfig config();
}
