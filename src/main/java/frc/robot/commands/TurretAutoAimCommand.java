package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.util.TurretAimingCalculator;
import frc.robot.util.TurretAimingCalculator.AimingParameters;

/**
 * Automatically aims the turret at the alliance hub.
 * 
 * <h2>WHAT THIS COMMAND DOES:</h2>
 * <ul>
 * <li>Rotates the TURRET to point at the hub center</li>
 * <li>Adjusts hood angle for distance</li>
 * <li>Sets flywheel speed for distance</li>
 * </ul>
 * 
 * <h2>WHAT THIS COMMAND DOES NOT DO:</h2>
 * <ul>
 * <li>Does NOT move the drivetrain/robot</li>
 * <li>Does NOT control robot driving at all</li>
 * </ul>
 * 
 * <h2>WHY DRIVETRAIN IS PASSED IN:</h2>
 * <p>
 * We need to READ the robot's position to calculate where to aim.
 * The drivetrain's pose estimator tells us "you are at (X, Y) on the field".
 * We then calculate "to hit the hub at (11.92, 4.03), aim the turret at angle
 * θ".
 * </p>
 * 
 * <h2>AIMING MODES:</h2>
 * <ol>
 * <li><b>Pose-Based (Primary)</b>: Use robot's known position → calculate exact
 * angle to hub</li>
 * <li><b>Vision Fallback</b>: If position uncertain → visually center on
 * AprilTag</li>
 * </ol>
 */
@Logged
public class TurretAutoAimCommand extends Command {

    // ==================
    // DEPENDENCIES
    // ==================

    /**
     * Drivetrain reference - READ ONLY.
     * We only call drivetrain.getState().Pose to get robot position.
     * We do NOT command any drivetrain movement.
     */
    private final CommandSwerveDrivetrain drivetrain;

    /** Vision subsystem - for fallback AprilTag targeting */
    private final Vision vision;

    /** Turret subsystem - controls turret rotation */
    private final Turret turret;

    /** Hood subsystem - controls hood angle */
    private final Hood hood;

    /** Shooter subsystem - controls flywheel speed */
    private final Shooter shooter;

    /** Calculator for aiming math */
    private final TurretAimingCalculator calculator;

    // ==================
    // TELEMETRY
    // ==================

    @Logged
    private boolean usingPoseMode = false;
    @Logged
    private double distanceToHubMeters = 0.0;
    @Logged
    private double targetTurretAngleDeg = 0.0;
    @Logged
    private double turretErrorDeg = 0.0;

    // ==================
    // TUNING CONSTANTS
    // ==================

    /** If pose ambiguity exceeds this, fall back to vision servo mode */
    private static final double POSE_CONFIDENCE_THRESHOLD = 0.3;

    /**
     * Proportional gain for vision servo mode (degrees adjustment per degree of
     * error)
     */
    private static final double VISION_SERVO_KP = 0.02;

    /** Tolerance for "aimed" state (degrees) */
    private static final double AIM_TOLERANCE_DEG = 2.0;

    /**
     * Creates a TurretAutoAimCommand.
     * 
     * @param drivetrain READ-ONLY - used to get robot's field position, NOT
     *                   controlled
     * @param vision     Used for fallback AprilTag targeting
     * @param turret     The turret we control (rotation)
     * @param hood       The hood we control (angle)
     * @param shooter    The shooter we control (flywheel)
     */
    public TurretAutoAimCommand(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            Turret turret,
            Hood hood,
            Shooter shooter) {

        this.drivetrain = drivetrain; // READ ONLY - for position
        this.vision = vision;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.calculator = new TurretAimingCalculator();

        // IMPORTANT: Require all three subsystems we control
        addRequirements(turret, hood, shooter);
    }

    @Override
    public void initialize() {
        calculator.clearAllianceCache();
    }

    @Override
    public void execute() {
        // ==================
        // STEP 1: Get robot's current position (READ from drivetrain, not controlling
        // it)
        // ==================
        Pose2d robotPose = drivetrain.getState().Pose; // Where am I on the field?

        // ==================
        // STEP 2: Choose aiming mode based on confidence
        // ==================
        boolean trustPose = isPoseReliable();

        if (trustPose) {
            // POSE-BASED AIMING: We know where we are, calculate exact angle
            aimUsingPose(robotPose);
            usingPoseMode = true;
        } else {
            // VISION FALLBACK: Don't trust position, just center on visible tag
            aimUsingVisionServo();
            usingPoseMode = false;
        }
    }

    /**
     * PRIMARY MODE: Aim turret using robot's known field position.
     * 
     * <p>
     * How it works:
     * <ol>
     * <li>Robot is at position (X, Y) on field</li>
     * <li>Hub is at position (11.92, 4.03) for Red alliance</li>
     * <li>Calculate angle: atan2(hubY - robotY, hubX - robotX)</li>
     * <li>Subtract robot heading to get turret-relative angle</li>
     * <li>Command turret to rotate to that angle</li>
     * </ol>
     */
    private void aimUsingPose(Pose2d robotPose) {
        // Calculate all aiming parameters from robot position
        AimingParameters params = calculator.calculate(robotPose);

        // Update telemetry
        distanceToHubMeters = params.distanceToHub();
        targetTurretAngleDeg = params.turretAngle().getDegrees();
        turretErrorDeg = turret.getTurretPosition().minus(params.turretAngle()).getDegrees();

        // Only command turret if parameters are valid (reasonable distance)
        if (params.isValid()) {
            // TURRET ROTATION: Point at hub center
            turret.moveTurret(Rotations.of(params.turretAngle().getRotations()));

            // HOOD ANGLE: Adjust for distance
            hood.setHoodAngle(Rotations.of(params.hoodAngle().getRotations()));

            // FLYWHEEL: Speed for distance (assuming RPM needs conversion)
            shooter.setShooterSpeed(RotationsPerSecond.of(params.flywheelRPM() / 60.0)); // Convert RPM to RPS
        }
    }

    /**
     * FALLBACK MODE: Aim turret by visually centering on AprilTag.
     * 
     * <p>
     * Used when we don't trust our position estimate.
     * Less accurate than pose-based, but works when lost.
     * </p>
     * 
     * <p>
     * How it works:
     * <ol>
     * <li>Turret camera sees an AprilTag</li>
     * <li>Tag appears 5° to the left in camera view</li>
     * <li>Rotate turret 5° left to center the tag</li>
     * <li>Repeat until tag is centered</li>
     * </ol>
     */
    private void aimUsingVisionServo() {
        var bestTarget = vision.getBestTarget();

        if (bestTarget.isPresent()) {
            int tagId = bestTarget.get().getFiducialId();

            // Only track hub tags (ignore tower/outpost tags)
            if (VisionConstants.isHubTag(tagId)) {
                // How many degrees is the tag off-center?
                double yawErrorDeg = bestTarget.get().getYaw();

                // Proportional adjustment (negative because positive yaw = target is left)
                Rotation2d adjustment = Rotation2d.fromDegrees(-yawErrorDeg * VISION_SERVO_KP);
                Angle currentPosition = turret.getTurretMotor().getPosition().getValue();
                Angle newSetpoint = currentPosition.plus(Rotations.of(adjustment.getRotations()));

                // Rotate turret to center the tag
                turret.moveTurret(newSetpoint);

                // Update telemetry
                turretErrorDeg = yawErrorDeg;
            }
        }

        // Unknown distance in vision mode
        distanceToHubMeters = 0.0;
    }

    /**
     * Checks if we can trust our pose estimate.
     */
    private boolean isPoseReliable() {
        var bestTarget = vision.getBestTarget();
        if (bestTarget.isEmpty()) {
            return false;
        }
        return bestTarget.get().getPoseAmbiguity() <= POSE_CONFIDENCE_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        // Don't stop motors - keep flywheel spinning between shots
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until button released
    }

    // ==================
    // PUBLIC STATUS METHODS
    // ==================

    /** Returns true if turret is aimed within tolerance. Check before shooting. */
    public boolean isAimed() {
        return Math.abs(turretErrorDeg) <= AIM_TOLERANCE_DEG;
    }

    /** Returns true if using accurate pose-based aiming (vs fallback). */
    public boolean isUsingPoseBasedAiming() {
        return usingPoseMode;
    }

    /** Returns distance to hub in meters (0 if unknown). */
    public double getDistanceToHub() {
        return distanceToHubMeters;
    }
}
