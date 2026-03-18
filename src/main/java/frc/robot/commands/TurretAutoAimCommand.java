package frc.robot.commands;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.util.TurretAimingCalculator;
import frc.robot.util.TurretAimingCalculator.AimingParameters;

/**
 * Production turret auto-aim with 3-state machine:
 *
 * <p><b>SEEKING</b> — No tag visible for a prolonged period ({@code COAST_TO_SEEK_SEC}).
 * Uses fused odometry pose to compute atan2(hub − robot) and pre-aims the turret
 * via MotionMagic (Slot 0) to get the camera's FOV onto the hub.
 *
 * <p><b>TRACKING</b> — Turret camera sees a hub tag. Uses closed-loop VelocityVoltage
 * (Slot 1) proportional to camera tx error.  Gyro feed-forward counteracts
 * robot rotation lag.
 *
 * <p><b>COASTING</b> — Tag was visible recently but dropped for a short time
 * (missed frames, vibration, brief occlusion).  Holds the turret at its last
 * known position via MotionMagic so the camera stays in the neighbourhood of
 * the tag.  Prevents the violent jerk that used to happen when the system
 * fell straight from TRACKING → SEEKING on a single missed frame.
 *
 * <p>No wrapping — turret clamps at soft limits.  When the robot rotates the
 * tag into the turret's dead zone the turret waits at the nearest limit until
 * the geometry brings the tag back into range.
 */
@Logged
public class TurretAutoAimCommand extends Command {

    // ================================================================
    //  STATE ENUM
    // ================================================================
    public enum AimState { SEEKING, TRACKING, COASTING }

    // ================================================================
    //  SUBSYSTEMS & CALCULATORS
    // ================================================================
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision; // nullable
    private final Turret turret;
    private final TurretAimingCalculator calculator;

    // ================================================================
    //  TELEMETRY (all @Logged for Elastic monitoring)
    // ================================================================
    @Logged(importance = Logged.Importance.CRITICAL) private String state = "SEEKING";
    @Logged(importance = Logged.Importance.CRITICAL) private double distanceToHubMeters = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double visionTxDeg = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean visionActive = false;
    @Logged(importance = Logged.Importance.CRITICAL) private int trackedTagId = -1;
    @Logged(importance = Logged.Importance.DEBUG) private double odomSetpointRot = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double commandedRPS = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double gyroFFVolts = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double coastHoldRot = 0.0;

    // ================================================================
    //  TUNING CONSTANTS
    // ================================================================
    // Vision tracking (TRACKING state)
    private static final double kTxToRPS = 0.06;        // RPS per degree of camera tx error
    private static final double MAX_TRACKING_RPS = 2.0;  // cap turret tracking speed
    private static final double DEADBAND_DEG = 1.0;      // don't correct below this

    // Gyro feedforward — counteracts robot rotation so turret holds on target
    private static final double kGyroFF = 0.25;           // volts per RPS of robot yaw rate

    // TRACKING → COASTING: persist last velocity this long after the last fresh frame,
    // then latch current position and hold.  Short = responsive, but prevents 1-frame drops.
    private static final double TRACKING_PERSIST_SEC = 0.15;

    // COASTING → SEEKING: hold position this long before giving up and using odometry.
    // Long enough to survive a brief occlusion or camera hiccup.
    private static final double COAST_TO_SEEK_SEC = 2.0;

    // Soft limit margin — stop tracking velocity within this distance of range limits
    private static final double SOFT_LIMIT_MARGIN = 0.03;

    // ================================================================
    //  INTERNAL STATE
    // ================================================================
    private final VelocityVoltage trackingVelocityRequest = new VelocityVoltage(0).withSlot(1);
    private AimState currentState = AimState.SEEKING;
    private double lastVisionTimestamp = 0.0;
    private double lastTagSeenTimeSec = 0.0;    // last time a hub tag was actually seen
    private double lastDesiredRPS = 0.0;
    private double coastPositionRot = 0.0;      // turret position when we entered COASTING

    // ================================================================
    //  CONSTRUCTOR
    // ================================================================
    public TurretAutoAimCommand(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            Turret turret) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.turret = turret;
        this.calculator = new TurretAimingCalculator();
        addRequirements(turret);
    }

    // ================================================================
    //  COMMAND LIFECYCLE
    // ================================================================
    @Override
    public void initialize() {
        calculator.clearAllianceCache();
        currentState = AimState.SEEKING;
        lastVisionTimestamp = 0.0;
        lastTagSeenTimeSec = 0.0;
        lastDesiredRPS = 0.0;
        coastPositionRot = 0.0;
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        // ============================================================
        //  ALWAYS: Compute odometry-based aiming parameters
        // ============================================================
        Pose2d robotPose = drivetrain.getState().Pose;
        AimingParameters params = calculator.calculate(robotPose);
        distanceToHubMeters = params.distanceToHub();

        odomSetpointRot = TurretConstants.TURRET_FORWARD_POSITION
                - params.turretAngle().getRotations()
                + TurretConstants.TURRET_AIM_OFFSET;

        // ============================================================
        //  ALWAYS: Process fresh vision frames
        // ============================================================
        visionActive = false;

        if (vision != null
                && vision.isTurretResultFresh()
                && vision.turretCameraHasTargets()) {
            double currentTimestamp = vision.getTurretResultTimestamp();
            if (currentTimestamp != lastVisionTimestamp) {
                lastVisionTimestamp = currentTimestamp;

                var target = vision.getTurretCameraBestTarget();
                if (target.isPresent()) {
                    int tagId = target.get().getFiducialId();

                    if (VisionConstants.BENCH_TEST_ANY_TAG || VisionConstants.isHubTag(tagId)) {
                        double tx = target.get().getYaw();
                        visionTxDeg = tx;
                        visionActive = true;
                        trackedTagId = tagId;
                        lastTagSeenTimeSec = now;

                        if (Math.abs(tx) > DEADBAND_DEG) {
                            lastDesiredRPS = MathUtil.clamp(kTxToRPS * tx, -MAX_TRACKING_RPS, MAX_TRACKING_RPS);
                        } else {
                            lastDesiredRPS = 0.0;
                        }
                    }
                }
            }
        }

        // ============================================================
        //  STATE TRANSITIONS
        // ============================================================
        double timeSinceTag = now - lastTagSeenTimeSec;
        boolean tagFresh = timeSinceTag < TRACKING_PERSIST_SEC;
        boolean tagRecent = timeSinceTag < COAST_TO_SEEK_SEC;

        switch (currentState) {
            case SEEKING:
                // Any fresh tag → jump straight to TRACKING
                if (tagFresh) {
                    currentState = AimState.TRACKING;
                }
                break;

            case TRACKING:
                if (!tagFresh) {
                    // Tag lost briefly → COASTING (hold position, don't jerk to odom)
                    coastPositionRot = turret.getTurretMotor().getPosition().getValueAsDouble();
                    currentState = AimState.COASTING;
                }
                break;

            case COASTING:
                if (tagFresh) {
                    // Tag re-acquired → back to TRACKING (smooth, no jerk)
                    currentState = AimState.TRACKING;
                } else if (!tagRecent) {
                    // Tag gone for >2s → give up, use odometry
                    currentState = AimState.SEEKING;
                }
                break;
        }
        state = currentState.name();
        coastHoldRot = coastPositionRot;

        // ============================================================
        //  STATE EXECUTION
        // ============================================================
        double turretPos = turret.getTurretMotor().getPosition().getValueAsDouble();

        switch (currentState) {
            case SEEKING: {
                // Odometry pre-aim via MotionMagic (Slot 0). Clamp to range.
                double clampedSetpoint = MathUtil.clamp(odomSetpointRot,
                        TurretConstants.TURRET_REVERSE_LIMIT, TurretConstants.TURRET_FORWARD_LIMIT);
                turret.moveTurret(Rotations.of(clampedSetpoint));
                commandedRPS = 0.0;
                break;
            }

            case TRACKING: {
                // Velocity control proportional to camera tx error (Slot 1).
                double desiredRPS = lastDesiredRPS;

                // Gyro feedforward: counteract robot rotation
                double robotYawRPS = drivetrain.getPigeon2()
                        .getAngularVelocityZWorld().getValueAsDouble() / 360.0;
                gyroFFVolts = kGyroFF * (-robotYawRPS);

                // Soft-limit clamp: don't spin into a limit
                if (turretPos > TurretConstants.TURRET_FORWARD_LIMIT - SOFT_LIMIT_MARGIN && desiredRPS > 0) {
                    desiredRPS = 0;
                }
                if (turretPos < TurretConstants.TURRET_REVERSE_LIMIT + SOFT_LIMIT_MARGIN && desiredRPS < 0) {
                    desiredRPS = 0;
                }

                commandedRPS = desiredRPS;
                turret.getTurretMotor().setControl(
                        trackingVelocityRequest.withVelocity(desiredRPS).withFeedForward(gyroFFVolts));
                break;
            }

            case COASTING: {
                // Hold the position we were at when the tag was lost.
                // No jerk — camera stays pointed at where the tag was.
                // Gyro FF keeps compensating for robot rotation while coasting.
                double robotYawRPS = drivetrain.getPigeon2()
                        .getAngularVelocityZWorld().getValueAsDouble() / 360.0;
                double coastFF = kGyroFF * (-robotYawRPS);
                turret.moveTurret(Rotations.of(coastPositionRot), coastFF);
                commandedRPS = 0.0;
                break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // ================================================================
    //  PUBLIC ACCESSORS
    // ================================================================
    public boolean isAimed() {
        if (currentState != AimState.TRACKING) return false;
        return Math.abs(visionTxDeg) < DEADBAND_DEG;
    }

    public double getDistanceToHub() {
        return distanceToHubMeters;
    }

    public TurretAimingCalculator getCalculator() {
        return calculator;
    }

    public AimState getCurrentState() {
        return currentState;
    }
}
