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
 * Production turret auto-aim with 2-state machine:
 *
 * <p><b>SEEKING</b> — No tag visible. Uses fused pose estimate (odometry + all camera
 * vision) to compute atan2(hub - robot) and pre-aims the turret via MotionMagic (Slot0).
 * Gets the turret camera's FOV pointed at the hub so it can acquire a tag.
 *
 * <p><b>TRACKING</b> — Turret camera sees a hub tag. Uses closed-loop VelocityVoltage
 * (Slot1) proportional to camera tx error. Motor PID automatically compensates for
 * cable tension and friction. Smooth, continuous, responsive.
 *
 * <p>Hood and shooter are NOT commanded here — they are handled by ShootCommand
 * when the operator pulls the right trigger. Gyro feedforward counteracts robot
 * rotation lag during TRACKING.
 */
@Logged
public class TurretAutoAimCommand extends Command {

    // ================================================================
    //  STATE ENUM
    // ================================================================
    public enum AimState { SEEKING, TRACKING }

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

    // ================================================================
    //  TUNING CONSTANTS
    // ================================================================
    // Vision tracking (TRACKING state)
    private static final double kTxToRPS = 0.06;       // RPS per degree of camera tx error
    private static final double MAX_TRACKING_RPS = 1.5; // cap turret tracking speed
    private static final double DEADBAND_DEG = 1.0;     // don't correct below this

    // Gyro feedforward
    private static final double kGyroFF = 0.15;         // volts per RPS of robot yaw rate

    // Target lost timeout — persist velocity for this long after last fresh frame
    private static final double TARGET_LOST_TIMEOUT_SEC = 0.25;

    // Soft limit margin — stop tracking velocity within this distance of range limits
    private static final double SOFT_LIMIT_MARGIN = 0.03;

    // ================================================================
    //  INTERNAL STATE
    // ================================================================
    private final VelocityVoltage trackingVelocityRequest = new VelocityVoltage(0).withSlot(1);
    private AimState currentState = AimState.SEEKING;
    private double lastVisionTimestamp = 0.0;
    private double lastTrackingTimeSec = 0.0;
    private double lastDesiredRPS = 0.0;

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
        lastTrackingTimeSec = 0.0;
        lastDesiredRPS = 0.0;
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

        // Turret setpoint from odometry: where should turret point based on fused pose?
        // TURRET_FORWARD_POSITION = 0.0 (turret zero = robot forward at boot)
        // turretAngle = field angle to hub minus robot heading (robot-relative)
        // Positive turret rotation = clockwise from above
        // WPILib atan2 returns CCW-positive angles, turret is CW-positive → negate
        odomSetpointRot = TurretConstants.TURRET_FORWARD_POSITION
                - params.turretAngle().getRotations()
                + TurretConstants.TURRET_AIM_OFFSET;

        // ============================================================
        //  ALWAYS: Process fresh vision frames
        // ============================================================
        visionActive = false;
        visionTxDeg = 0.0;

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
                        lastTrackingTimeSec = now;

                        // Compute tracking velocity from fresh tx
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
        boolean recentlyTracking = (now - lastTrackingTimeSec) < TARGET_LOST_TIMEOUT_SEC;

        switch (currentState) {
            case SEEKING:
                if (recentlyTracking) {
                    currentState = AimState.TRACKING;
                }
                break;
            case TRACKING:
                if (!recentlyTracking) {
                    currentState = AimState.SEEKING;
                }
                break;
        }
        state = currentState.name();

        // ============================================================
        //  STATE EXECUTION
        // ============================================================
        double turretPos = turret.getTurretMotor().getPosition().getValueAsDouble();

        switch (currentState) {
            case SEEKING: {
                // Use odometry to pre-aim turret toward hub via MotionMagic (Slot0).
                // Clamp to turret range — if hub is in dead zone, go to nearest limit.
                double clampedSetpoint = MathUtil.clamp(odomSetpointRot,
                        TurretConstants.TURRET_REVERSE_LIMIT, TurretConstants.TURRET_FORWARD_LIMIT);
                turret.moveTurret(Rotations.of(clampedSetpoint));
                commandedRPS = 0.0;
                break;
            }
            case TRACKING: {
                // Velocity control proportional to camera tx error (Slot1).
                // Motor PID handles cable friction automatically.
                double desiredRPS = lastDesiredRPS;

                // Gyro feedforward: counteract robot rotation lag
                double robotYawRPS = drivetrain.getPigeon2()
                        .getAngularVelocityZWorld().getValueAsDouble() / 360.0;
                gyroFFVolts = kGyroFF * (-robotYawRPS);

                // Soft-limit safety: don't spin toward a limit we're already near
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
