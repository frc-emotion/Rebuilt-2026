package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Constants.VisionConstants;

/**
 * Calculates turret aiming parameters based on robot pose and target position.
 *
 * Uses two {@link InterpolatingDoubleTreeMap} tables keyed by distance (meters):
 * <ul>
 *   <li><b>Hood angle</b> (mechanism rotations, 0.0–0.08)</li>
 *   <li><b>Flywheel speed</b> (RPS — rotations per second)</li>
 * </ul>
 *
 * <h3>How the interpolation tables work</h3>
 * {@code InterpolatingDoubleTreeMap} is WPILib's piecewise-linear lookup.
 * You insert (key, value) pairs — here key = distance in meters, value = hood
 * rotations or shooter RPS.  Between measured points the map linearly
 * interpolates; outside the range it clamps to the nearest endpoint.
 *
 * <h3>How to collect data (calibration procedure)</h3>
 * <ol>
 *   <li>Place the robot centered on the hub at a known distance (use a tape
 *       measure from bumper to hub base).</li>
 *   <li>With the turret aimed dead-center, manually adjust hood angle and
 *       flywheel speed until the ball consistently scores.</li>
 *   <li>Record (distance, hoodRotations, shooterRPS) in the tables below.</li>
 *   <li>Repeat at 0.5 m increments from 1 m to 6 m (≈11 data points).
 *       More points near the transition zone (2–4 m) improve accuracy.
 *       Minimum viable: 5–6 points across the range.</li>
 *   <li>The map auto-interpolates between your measured points, so you do NOT
 *       need a measurement at every possible distance.</li>
 * </ol>
 */
public class TurretAimingCalculator {

    private final InterpolatingDoubleTreeMap flywheelRPSTable;
    private final InterpolatingDoubleTreeMap hoodAngleTable;

    private Alliance cachedAlliance = null;

    /**
     * All outputs needed to command the superstructure for a given robot pose.
     */
    public record AimingParameters(
            Rotation2d fieldRelativeAngle,
            Rotation2d turretAngle,
            double distanceToHub,
            double flywheelRPS,
            Rotation2d hoodAngle,
            boolean isValid) {
    }

    public TurretAimingCalculator() {
        // ── Flywheel speed table (distance m → RPS) ──────────────────────
        // Calibrated 2026-03-17 from field measurements (inches → meters).
        flywheelRPSTable = new InterpolatingDoubleTreeMap();
        flywheelRPSTable.put(1.232, 50.0);  //  48.5"
        flywheelRPSTable.put(2.216, 55.0);  //  87.25"
        flywheelRPSTable.put(3.232, 60.0);  // 127.25"
        flywheelRPSTable.put(4.191, 65.0);  // 165"
        flywheelRPSTable.put(5.207, 68.0);  // 205"
        flywheelRPSTable.put(6.223, 74.0);  // 245"
        flywheelRPSTable.put(7.239, 79.0);  // 285"

        // ── Hood angle table (distance m → mechanism rotations 0.0–0.08) ─
        // Calibrated 2026-03-17 from field measurements (inches → meters).
        hoodAngleTable = new InterpolatingDoubleTreeMap();
        hoodAngleTable.put(1.232, 0.000);   //  48.5"
        hoodAngleTable.put(2.216, 0.016);   //  87.25"
        hoodAngleTable.put(3.232, 0.031);   // 127.25"
        hoodAngleTable.put(4.191, 0.046);   // 165"
        hoodAngleTable.put(5.207, 0.050);   // 205"
        hoodAngleTable.put(6.223, 0.053);   // 245"
        hoodAngleTable.put(7.239, 0.055);   // 285"
    }

    /**
     * Gets the target hub center position based on current alliance.
     * 
     * @return Hub center position in field coordinates
     */
    public Translation2d getTargetHubCenter() {
        // Cache alliance to avoid repeated DS calls
        if (cachedAlliance == null) {
            cachedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        }

        return cachedAlliance == Alliance.Red
                ? VisionConstants.RED_HUB_CENTER
                : VisionConstants.BLUE_HUB_CENTER;
    }

    /**
     * Clears the cached alliance. Call this on alliance change or mode transitions.
     */
    public void clearAllianceCache() {
        cachedAlliance = null;
    }

    /**
     * Calculate all aiming parameters for the current robot pose.
     * 
     * @param robotPose Current robot pose on the field
     * @return Complete aiming parameters including turret angle, flywheel RPM, etc.
     */
    public AimingParameters calculate(Pose2d robotPose) {
        Translation2d hubCenter = getTargetHubCenter();
        Translation2d robotPosition = robotPose.getTranslation();
        Rotation2d robotHeading = robotPose.getRotation();

        // Calculate vector from robot to hub
        Translation2d robotToHub = hubCenter.minus(robotPosition);

        // Calculate field-relative angle to hub
        Rotation2d fieldAngle = new Rotation2d(robotToHub.getX(), robotToHub.getY());

        // Convert to turret-relative (turret angle when turret forward = robot forward)
        // Turret angle = field angle - robot heading
        Rotation2d turretAngle = fieldAngle.minus(robotHeading);

        // Normalize turret angle to [-180°, 180°]
        turretAngle = normalizeAngle(turretAngle);

        // Calculate distance to hub
        double distance = robotToHub.getNorm();

        // Look up shot parameters from interpolation tables
        double flywheelRPS = flywheelRPSTable.get(distance);
        double hoodAngleRot = hoodAngleTable.get(distance);
        Rotation2d hoodAngle = Rotation2d.fromRotations(hoodAngleRot);

        boolean isValid = distance >= 1.0 && distance <= 7.0;

        return new AimingParameters(
                fieldAngle,
                turretAngle,
                distance,
                flywheelRPS,
                hoodAngle,
                isValid);
    }

    /** Look up flywheel RPS from the interpolation table for a given distance. */
    public double getFlywheelRPS(double distanceMeters) {
        return flywheelRPSTable.get(distanceMeters);
    }

    /** Look up hood angle (mechanism rotations) from the interpolation table for a given distance. */
    public double getHoodAngleRot(double distanceMeters) {
        return hoodAngleTable.get(distanceMeters);
    }

    /**
     * Normalize angle to [-180°, 180°] range.
     */
    private Rotation2d normalizeAngle(Rotation2d angle) {
        double radians = angle.getRadians();
        while (radians > Math.PI)
            radians -= 2 * Math.PI;
        while (radians < -Math.PI)
            radians += 2 * Math.PI;
        return new Rotation2d(radians);
    }

}
