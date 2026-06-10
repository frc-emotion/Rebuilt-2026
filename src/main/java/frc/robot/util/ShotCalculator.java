package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Distance → shot parameter lookup. Replaces the legacy TurretAimingCalculator; the tables are
 * copied verbatim (calibrated 2026-03-17 via CalibrationCommand field measurements).
 *
 * <p>Calibration procedure: hold the calibration binding, set hood/RPS over NetworkTables under
 * /Calibration until the ball scores, record the echoed distance, and transcribe the
 * (distance, hoodRotations, shooterRPS) triple into the tables below.
 */
public final class ShotCalculator {
  // Multiplier on the closing-velocity distance correction (legacy value, unchanged).
  private static final double kShootWhileMovingMultiplier = 0.5;

  // Shoot-while-moving never worked in the legacy code (computed but unused); rebuilt here behind
  // a dashboard toggle, default OFF per team decision D1.
  private final BooleanEntry shootWhileMovingEnabled =
      NetworkTableInstance.getDefault()
          .getTable("Tuning")
          .getBooleanTopic("ShootWhileMovingEnabled")
          .getEntry(false);

  private final InterpolatingDoubleTreeMap flywheelRpsTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodAngleTable = new InterpolatingDoubleTreeMap();

  public ShotCalculator() {
    // Force-off at boot so the toggle is visible on the dashboard and a mid-session toggle
    // never silently survives a code restart.
    shootWhileMovingEnabled.set(false);

    // ── Flywheel speed table (distance m → RPS) ──────────────────────
    // Calibrated 2026-03-17 from field measurements (inches → meters).
    flywheelRpsTable.put(1.5501340177286882, 41.5); // 44//45 //42.5);  //  48.5"
    flywheelRpsTable.put(2.47698, 43.5); // 46 //47 //45  //  87.25"
    flywheelRpsTable.put(3.5, 48.5); // 51 //52 //50  // 127.25"
    flywheelRpsTable.put(4.620237, 56.5); // 58//59//59 //57 // 165"
    // flywheelRpsTable.put(5.207, 68.0);  // 205"
    // flywheelRpsTable.put(6.223, 74.0);  // 245"
    // flywheelRpsTable.put(7.239, 79.0);  // 285"

    // ── Hood angle table (distance m → mechanism rotations 0.0–0.08) ─
    // Calibrated 2026-03-17 from field measurements (inches → meters).
    hoodAngleTable.put(1.5501340177286882, 0.000); //  48.5"
    hoodAngleTable.put(2.47698, 0.02); //  87.25"
    hoodAngleTable.put(3.5, 0.03); // 127.25"
    hoodAngleTable.put(4.620237, 0.035); // 165"
    // hoodAngleTable.put(5.207, 0.050);   // 205"
    // hoodAngleTable.put(6.223, 0.053);   // 245"
    // hoodAngleTable.put(7.239, 0.055);   // 285"
  }

  public double getFlywheelRps(double distanceMeters) {
    return flywheelRpsTable.get(distanceMeters);
  }

  public double getHoodAngleRot(double distanceMeters) {
    return hoodAngleTable.get(distanceMeters);
  }

  /** Legacy validity window: shots are attempted between 1.0 and 7.0 meters. */
  public boolean isValidDistance(double distanceMeters) {
    return distanceMeters >= 1.0 && distanceMeters <= 7.0;
  }

  /**
   * Shoot-while-moving distance correction: project the robot-relative chassis velocity onto the
   * turret heading to get the closing speed toward the hub, and shorten the lookup distance
   * proportionally. Identity when the toggle is off (the default).
   */
  public double effectiveDistance(double distanceMeters, ChassisSpeeds speeds, double turretAngleRad) {
    if (!shootWhileMovingEnabled.get(false)) {
      return distanceMeters;
    }
    double closingVelocity =
        speeds.vxMetersPerSecond * Math.cos(turretAngleRad)
            + speeds.vyMetersPerSecond * Math.sin(turretAngleRad);
    return distanceMeters - closingVelocity * kShootWhileMovingMultiplier;
  }
}
