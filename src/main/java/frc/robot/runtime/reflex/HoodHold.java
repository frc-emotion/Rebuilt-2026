package frc.robot.runtime.reflex;

import java.util.OptionalDouble;

/**
 * The hood capture-once hold (W32). Entering a resting skill (IDLE / INTAKING) captures the current
 * hood position ONCE and holds it (deliberate change from the old sag-follow). PASS_AIMING and
 * CLEARING leave the hood UNCOMMANDED so it holds its last MotionMagic target. While shooting the
 * hood tracks a source. This reflex owns the one-shot capture: it re-captures only on a fresh entry
 * into the capture mode, exactly as the legacy onStateEntry did.
 */
public final class HoodHold {

  public enum Mode {
    /** Track a computed setpoint each loop (SHOOTING / PASSING). */
    SOURCE,
    /** Capture-once on entry, then hold (IDLE / INTAKING). */
    HOLD_CAPTURE,
    /** Do not command the hood; it holds its last target (PASS_AIMING / CLEARING). */
    UNCOMMANDED
  }

  private Mode lastMode = Mode.UNCOMMANDED;
  private double capturedRot = 0.0;

  public void reset() {
    lastMode = Mode.UNCOMMANDED;
  }

  /**
   * Resolve the hood command for this loop.
   *
   * @return the rotations to command in HOLD_CAPTURE mode; empty in SOURCE/UNCOMMANDED (SOURCE
   *     means the interpreter commands the computed setpoint; UNCOMMANDED means leave the hood
   *     untouched).
   */
  public OptionalDouble update(Mode mode, double currentHoodRot) {
    OptionalDouble command;
    if (mode == Mode.HOLD_CAPTURE) {
      if (lastMode != Mode.HOLD_CAPTURE) {
        capturedRot = currentHoodRot; // capture once on entry
      }
      command = OptionalDouble.of(capturedRot);
    } else {
      command = OptionalDouble.empty();
    }
    lastMode = mode;
    return command;
  }
}
