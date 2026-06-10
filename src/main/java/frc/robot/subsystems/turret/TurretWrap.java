package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;

/**
 * The turret wrap, as a pure function (legacy wrapAndClamp, verbatim semantics — W1).
 *
 * <p>The turret has ~403° of cable-limited travel. When a setpoint exceeds a limit, add or
 * subtract exactly one full rotation to reach the same physical heading from the other side of the
 * wind-up range, then clamp as the final safety net (needed because total travel is less than
 * 720°, so the wrapped equivalent can still be out of range). Only a single ±1 correction is
 * attempted, and the decision looks only at the setpoint, never the current position — both are
 * legacy semantics that must survive exactly.
 */
public final class TurretWrap {
  private TurretWrap() {}

  public record WrapResult(double commandedRot, boolean wrapped) {}

  public static WrapResult apply(double requestedRot, double reverseLimitRot, double forwardLimitRot) {
    double rot = requestedRot;
    if (rot < reverseLimitRot) {
      rot += 1.0;
    } else if (rot > forwardLimitRot) {
      rot -= 1.0;
    }
    rot = MathUtil.clamp(rot, reverseLimitRot, forwardLimitRot);
    return new WrapResult(rot, rot != requestedRot);
  }
}
