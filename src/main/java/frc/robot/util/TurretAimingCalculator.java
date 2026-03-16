package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Constants.VisionConstants;

/**
 * Calculates turret aiming parameters based on robot pose and target position.
 * 
 * <p>
 * This class handles the "position-based offset" problem by aiming at the hub
 * center
 * position rather than the AprilTag center. This automatically produces the
 * correct
 * angular offset based on robot position:
 * <ul>
 * <li>Robot on LEFT of hub → turret aims LEFT of tag centerline</li>
 * <li>Robot on RIGHT of hub → turret aims RIGHT of tag centerline</li>
 * <li>Robot in CENTER → turret aims at tag center</li>
 * </ul>
 * 
 * <p>
 * Usage:
 * 
 * <pre>{@code
 * TurretAimingCalculator calculator = new TurretAimingCalculator();
 * AimingParameters params = calculator.calculate(robotPose, robotHeading);
 * turret.setAngle(params.turretAngle());
 * }</pre>
 */
public class TurretAimingCalculator {

    // Interpolation tables for distance-based shot parameters
    private final InterpolatingDoubleTreeMap flywheelRPMTable;
    private final InterpolatingDoubleTreeMap hoodAngleTable;

    // Cached alliance for hub selection
    private Alliance cachedAlliance = null;

    /**
     * Container for all calculated aiming parameters.
     */
    public record AimingParameters(
            /** Field-relative angle to hub center (radians) */
            Rotation2d fieldRelativeAngle,

            /** Turret angle relative to robot (radians, 0 = forward) */
            Rotation2d turretAngle,

            /** Distance from robot to hub center (meters) */
            double distanceToHub,

            /** Calculated flywheel RPM for this distance */
            double flywheelRPM,

            /** Calculated hood angle for this distance */
            Rotation2d hoodAngle,

            /** Whether the target hub is valid based on alliance */
            boolean isValid) {
    }

    /**
     * Creates a new TurretAimingCalculator with default shot parameter tables.
     * 
     * <p>
     * TODO: Tune these interpolation values based on testing
     */
    public TurretAimingCalculator() {
        // Initialize flywheel RPM interpolation table (distance in meters -> RPM)
        // RPM / 60 = RPS sent to shooter.
        // TODO: populate with real tested values
        flywheelRPMTable = new InterpolatingDoubleTreeMap();
        flywheelRPMTable.put(1.0, 1200.0); // 1m -> 20 RPS (placeholder)
        flywheelRPMTable.put(2.0, 1200.0); // 2m -> 20 RPS (placeholder)
        flywheelRPMTable.put(3.0, 1200.0); // 3m -> 20 RPS (placeholder)
        flywheelRPMTable.put(4.0, 1200.0); // 4m -> 20 RPS (placeholder)
        flywheelRPMTable.put(5.0, 1200.0); // 5m -> 20 RPS (placeholder)
        flywheelRPMTable.put(6.0, 1200.0); // 6m -> 20 RPS (placeholder)

        // Initialize hood angle interpolation table (distance in meters -> mechanism rotations)
        // Hood range: 0.0 (flat) to 0.08 (max angle). Zeroed at startup.
        // Close = high angle, far = low angle.
        // TODO: populate with real tested values at each distance
        hoodAngleTable = new InterpolatingDoubleTreeMap();
        hoodAngleTable.put(1.0, 0.060); // 1m (placeholder)
        hoodAngleTable.put(2.0, 0.050); // 2m (placeholder)
        hoodAngleTable.put(3.0, 0.040); // 3m (placeholder)
        hoodAngleTable.put(4.0, 0.030); // 4m (placeholder)
        hoodAngleTable.put(5.0, 0.020); // 5m (placeholder)
        hoodAngleTable.put(6.0, 0.010); // 6m (placeholder)
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

        // Look up shot parameters
        double flywheelRPM = flywheelRPMTable.get(distance);
        double hoodAngleRot = hoodAngleTable.get(distance); // mechanism rotations (0-0.08)
        Rotation2d hoodAngle = Rotation2d.fromRotations(hoodAngleRot);

        // Validate - ensure we're not too far or too close
        boolean isValid = distance >= 1.0 && distance <= 7.0;

        return new AimingParameters(
                fieldAngle,
                turretAngle,
                distance,
                flywheelRPM,
                hoodAngle,
                isValid);
    }

    /**
     * Calculate ONLY the turret angle for quick aiming updates.
     * 
     * @param robotPose Current robot pose
     * @return Turret angle relative to robot (0 = forward)
     */
    public Rotation2d calculateTurretAngle(Pose2d robotPose) {
        Translation2d hubCenter = getTargetHubCenter();
        Translation2d robotToHub = hubCenter.minus(robotPose.getTranslation());
        Rotation2d fieldAngle = new Rotation2d(robotToHub.getX(), robotToHub.getY());
        return normalizeAngle(fieldAngle.minus(robotPose.getRotation()));
    }

    /**
     * Calculate distance to the current target hub.
     * 
     * @param robotPose Current robot pose
     * @return Distance in meters
     */
    public double calculateDistance(Pose2d robotPose) {
        Translation2d hubCenter = getTargetHubCenter();
        return hubCenter.minus(robotPose.getTranslation()).getNorm();
    }

    /**
     * Calculate the dynamic turret camera transform based on current turret angle.
     * 
     * <p>
     * The turret camera position changes relative to robot center as the turret
     * rotates:
     * 
     * <pre>
     * Robot_to_TurretCam = Robot_to_TurretPivot ⊕ TurretRotation ⊕ TurretPivot_to_Camera
     * </pre>
     * 
     * @param turretAngle Current turret angle from encoder (0 = turret forward
     *                    matches robot forward)
     * @return Transform3d from robot center to turret camera
     */
    public static Transform3d calculateTurretCameraTransform(Rotation2d turretAngle) {
        // Get static transforms from constants
        Transform3d robotToTurretPivot = VisionConstants.ROBOT_TO_TURRET_PIVOT;
        Transform3d turretPivotToCam = VisionConstants.TURRET_PIVOT_TO_CAM;

        // Create rotation transform for current turret angle (rotation around Z axis)
        Transform3d turretRotation = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, 0, turretAngle.getRadians()));

        // Compose the transforms: robot → pivot → rotation → camera
        return robotToTurretPivot.plus(turretRotation).plus(turretPivotToCam);
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

    /**
     * Check if a given turret angle is within safe rotation limits.
     * 
     * @param angle    Turret angle to check
     * @param minAngle Minimum safe angle (degrees)
     * @param maxAngle Maximum safe angle (degrees)
     * @return true if angle is within limits
     */
    public static boolean isWithinLimits(Rotation2d angle, double minAngle, double maxAngle) {
        double degrees = angle.getDegrees();
        return degrees >= minAngle && degrees <= maxAngle;
    }
}
