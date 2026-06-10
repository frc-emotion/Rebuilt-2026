package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 * Teleop and path-following values, copied verbatim from legacy RobotContainer and
 * configurePathPlanner. Tuned numbers are sacred.
 *
 * <p>NOTE: PathPlanner loads robot mass/MOI/module config from deploy/pathplanner/settings.json via
 * RobotConfig.fromGUISettings() — that file is a second source of truth alongside TunerConstants
 * (settings.json says max 5.44 m/s vs kSpeedAt12Volts 5.85 m/s, known mismatch).
 */
public final class DriveConstants {
  private DriveConstants() {}

  public static final double kMaxSpeedMps =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // 5.85 m/s

  public static final double kMaxAngularRateRadPerSec =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 4.712 rad/s

  // Deadbands applied INSIDE the SwerveRequest (legacy behavior), not on raw joystick values.
  public static final double kTranslationDeadband = kMaxSpeedMps * 0.05;
  public static final double kRotationDeadband = kMaxAngularRateRadPerSec * 0.1;

  // PathPlanner holonomic controller gains — starting defaults, tune on carpet (legacy comment).
  public static final double kPathTranslationP = 10.0;
  public static final double kPathRotationP = 7.0;
}
