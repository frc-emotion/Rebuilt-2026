package frc.robot.runtime.reflex;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.runtime.Mechanisms;

/**
 * The non-data, always-local real-time safety floor that is not a per-skill behavior:
 *
 * <ul>
 *   <li><b>Disabled-output lockout</b> — when the robot is disabled, every mechanism is forced
 *       neutral (the interpreter's per-skill commands are not actuated). Called from the robot's
 *       disabled hooks.
 *   <li><b>Brownout tolerance</b> — surfaced as a flag for telemetry; the actual brownout
 *       protection is the per-mechanism stator/supply current limits, which now live in
 *       mechanisms.json and are applied to every Talon by the mechanism layer (so the tolerance is
 *       preserved as data).
 *   <li><b>Motor-safety / closed-loop on the Talons</b> — velocity/position control runs onboard
 *       the Talons (Phoenix 6), as before; this stays local and is never delegated to a
 *       coprocessor.
 * </ul>
 */
public final class SafetyReflex {
  private SafetyReflex() {}

  /** Force every mechanism to neutral output (disabled lockout). */
  public static void lockout(Mechanisms mechanisms) {
    mechanisms.stopAll();
  }

  public static boolean isDisabled() {
    return DriverStation.isDisabled();
  }

  public static boolean isBrownedOut() {
    return RobotController.isBrownedOut();
  }
}
