package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.util.TurretAimingCalculator;

/**
 * Unified turret command with 2-state machine:
 *
 * <p><b>MANUAL</b> — No hub tag visible. Operator controls turret via joystick.
 * This is the default state.
 *
 * <p><b>TRACKING</b> — Turret camera sees a hub tag. Uses closed-loop VelocityVoltage
 * (Slot 1) proportional to camera tx error. Gyro feed-forward counteracts
 * robot rotation lag. Distance from camera feeds interpolation tables for
 * hood angle and shooter speed.
 *
 * <p>Transition MANUAL → TRACKING: instant when a hub tag is detected.
 * Transition TRACKING → MANUAL: after 0.15s of no tag (prevents single-frame drops).
 *
 * <p>SEEKING (odometry pre-aim) and COASTING are commented out for now.
 *
 * <p>No wrapping — turret clamps at soft limits.
 */
@Logged
public class TurretAutoAimCommand extends Command {

    // ================================================================
    //  STATE ENUM
    // ================================================================
    public enum AimState {
        MANUAL,     // operator joystick controls turret
        TRACKING,   // vision auto-tracking a hub tag
        // SEEKING,  // odometry pre-aim — commented out
        // COASTING, // hold position after tag loss — commented out
    }

    // ================================================================
    //  SUBSYSTEMS & INPUTS
    // ================================================================
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision; // nullable
    private final Turret turret;
    private final TurretAimingCalculator calculator;
    private final DoubleSupplier joystickSupplier;

    // ================================================================
    //  TELEMETRY (all @Logged for Elastic monitoring)
    //  Operator: check "visionActive" or "state" on Elastic dashboard
    //  to know whether the turret is auto-tracking or awaiting manual input.
    // ================================================================
    @Logged(importance = Logged.Importance.CRITICAL) private String state = "MANUAL";
    @Logged(importance = Logged.Importance.CRITICAL) private double distanceToHubMeters = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double visionTxDeg = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean visionActive = false;
    @Logged(importance = Logged.Importance.CRITICAL) private int trackedTagId = -1;
    @Logged(importance = Logged.Importance.DEBUG) private double commandedRPS = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double gyroFFVolts = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double leadAngleDeg = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double effectiveDistanceMeters = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double robotSpeedMPS = 0.0;

    // ================================================================
    //  TUNING CONSTANTS
    // ================================================================
    // Vision tracking (TRACKING state)
    private static final double kTxToRPS = 0.12;        // RPS per degree of camera tx error
    private static final double MAX_TRACKING_RPS = 2.0;  // cap turret tracking speed
    private static final double DEADBAND_DEG = 0.5;      // don't correct below this
    private static final double MIN_TRACKING_RPS = 0.25;  // minimum velocity to overcome friction

    // Gyro feedforward — counteracts robot rotation so turret holds on target
    private static final double kGyroFF = 0.25;           // volts per RPS of robot yaw rate

    // TRACKING → MANUAL: persist tracking this long after last fresh frame.
    // Prevents single-frame drops from jerking back to manual.
    private static final double TRACKING_PERSIST_SEC = 0.15;

    // Manual joystick deadband
    private static final double MANUAL_DEADBAND = 0.08;

    // Soft limit margin — stop tracking velocity within this distance of range limits
    private static final double SOFT_LIMIT_MARGIN = 0.03;

    // Moving shot compensation — offsets turret aim and effective distance
    // to account for robot velocity during ball flight.
    private static final boolean MOVING_SHOT_ENABLED = true;
    private static final double BALL_EXIT_SPEED_MPS = 12.0; // approximate ball speed (tune on robot)
    private static final double MOVING_SHOT_GAIN = 1.0;     // 0 = disabled, 1 = full compensation

    // ================================================================
    //  INTERNAL STATE
    // ================================================================
    private final VelocityVoltage trackingVelocityRequest = new VelocityVoltage(0).withSlot(1);
    private AimState currentState = AimState.MANUAL;
    private double lastVisionTimestamp = 0.0;
    private double lastTagSeenTimeSec = 0.0;    // last time a hub tag was actually seen
    private double lastDesiredRPS = 0.0;
    private double lastCameraDistance = 0.0;    // last distance from camera (persists after tag loss)
    private double lastAdjustedTx = 0.0;        // lead-compensated tx (used by isAimed)

    // ================================================================
    //  CONSTRUCTOR
    // ================================================================
    /**
     * Creates a unified turret command.
     *
     * @param drivetrain    swerve drivetrain (for gyro FF)
     * @param vision        vision subsystem (turret camera)
     * @param turret        turret subsystem
     * @param joystickSupplier  operator right-X axis for manual turret control
     */
    public TurretAutoAimCommand(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            Turret turret,
            DoubleSupplier joystickSupplier) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.turret = turret;
        this.calculator = new TurretAimingCalculator();
        this.joystickSupplier = joystickSupplier;
        addRequirements(turret);
    }

    // ================================================================
    //  COMMAND LIFECYCLE
    // ================================================================
    @Override
    public void initialize() {
        calculator.clearAllianceCache();
        currentState = AimState.MANUAL;
        lastVisionTimestamp = 0.0;
        lastTagSeenTimeSec = 0.0;
        lastDesiredRPS = 0.0;
        lastCameraDistance = 0.0;
        lastAdjustedTx = 0.0;
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        // ============================================================
        //  Step 1: Capture fresh vision frames (raw tx + distance)
        // ============================================================
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
                        visionTxDeg = target.get().getYaw();
                        trackedTagId = tagId;
                        lastTagSeenTimeSec = now;
                        lastCameraDistance = vision.getTurretCameraDistanceToTarget();
                    }
                }
            }
        }

        // ============================================================
        //  Step 2: Moving shot compensation (runs every cycle)
        //  Decomposes robot velocity into turret-relative radial/tangential,
        //  then computes a lead angle offset and effective distance.
        //  At rest (v=0): lead=0, effectiveDist=cameraDistance → standard shot.
        // ============================================================
        double leadOffsetDeg = 0.0;
        double effectiveDist = lastCameraDistance;

        if (MOVING_SHOT_ENABLED && lastCameraDistance > 0.5) {
            var speeds = drivetrain.getState().Speeds;

            // Turret aiming direction in WPILib convention (CCW+).
            // Motor positive = CW, so negate for WPILib.
            double turretDirRad = -turret.getTurretPosition().getRadians();
            double aimCos = Math.cos(turretDirRad);
            double aimSin = Math.sin(turretDirRad);

            // ChassisSpeeds: vx = forward (+X), vy = left (+Y) — robot-relative
            // Project onto turret aiming axis and its perpendicular
            double vRadial = speeds.vxMetersPerSecond * aimCos
                    + speeds.vyMetersPerSecond * aimSin;     // positive = approaching hub
            double vTangential = -speeds.vxMetersPerSecond * aimSin
                    + speeds.vyMetersPerSecond * aimCos;     // positive = moving left rel. to aim

            double flightTime = lastCameraDistance / BALL_EXIT_SPEED_MPS;

            // Turret lead angle: compensate for lateral ball drift during flight.
            // Robot moves left (vTangential>0) → ball drifts left → aim turret RIGHT
            // → tag appears LEFT of camera center → negative leadOffsetDeg.
            // At steady state: visionTxDeg = leadOffsetDeg (negative) → turret right of tag ✓
            leadOffsetDeg = -Math.toDegrees(
                    Math.atan2(vTangential * flightTime, lastCameraDistance))
                    * MOVING_SHOT_GAIN;

            // Effective distance: if approaching hub, ball needs less energy.
            effectiveDist = lastCameraDistance - vRadial * flightTime;
            effectiveDist = MathUtil.clamp(effectiveDist, 1.0, 8.0);

            robotSpeedMPS = Math.sqrt(
                    speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
                    + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
        } else {
            robotSpeedMPS = 0.0;
        }

        leadAngleDeg = leadOffsetDeg;
        effectiveDistanceMeters = effectiveDist;
        distanceToHubMeters = effectiveDist;

        // ============================================================
        //  Step 3: Compute adjusted tx and tracking velocity (every cycle)
        //  Uses lead-compensated tx so the turret aims ahead of the target
        //  when the robot is moving, and exactly at the target when stationary.
        // ============================================================
        double adjustedTx = visionTxDeg - leadOffsetDeg;
        lastAdjustedTx = adjustedTx;

        double timeSinceTag = now - lastTagSeenTimeSec;
        boolean tagFresh = lastTagSeenTimeSec > 0 && timeSinceTag < TRACKING_PERSIST_SEC;
        visionActive = tagFresh;

        if (tagFresh) {
            if (Math.abs(adjustedTx) > DEADBAND_DEG) {
                double rawRPS = kTxToRPS * (-adjustedTx);
                // Ensure minimum velocity to overcome turret friction/cable drag
                if (Math.abs(rawRPS) < MIN_TRACKING_RPS) {
                    rawRPS = Math.copySign(MIN_TRACKING_RPS, rawRPS);
                }
                lastDesiredRPS = MathUtil.clamp(rawRPS, -MAX_TRACKING_RPS, MAX_TRACKING_RPS);
            } else {
                lastDesiredRPS = 0.0;
            }
        }

        // ============================================================
        //  Step 4: State transitions
        // ============================================================
        switch (currentState) {
            case MANUAL:
                if (tagFresh) {
                    currentState = AimState.TRACKING;
                }
                break;

            case TRACKING:
                if (!tagFresh) {
                    currentState = AimState.MANUAL;
                }
                break;

            // SEEKING and COASTING commented out — not used in this architecture
            // case SEEKING: ...
            // case COASTING: ...
        }
        state = currentState.name();

        // ============================================================
        //  Step 5: State execution
        // ============================================================
        double turretPos = turret.getTurretMotor().getPosition().getValueAsDouble();

        switch (currentState) {
            case MANUAL: {
                // Pass through operator joystick input
                double input = MathUtil.applyDeadband(joystickSupplier.getAsDouble(), MANUAL_DEADBAND);
                double clampedInput = MathUtil.clamp(input, -1, 1);
                turret.setTurretVoltage(clampedInput);
                commandedRPS = 0.0;
                gyroFFVolts = 0.0;
                break;
            }

            case TRACKING: {
                double desiredRPS = lastDesiredRPS;

                // Gyro feedforward: counteract robot rotation
                double robotYawRPS = drivetrain.getPigeon2()
                        .getAngularVelocityZWorld().getValueAsDouble() / 360.0;
                gyroFFVolts = kGyroFF * robotYawRPS;

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
    /** Returns true when TRACKING and lead-compensated tx is within deadband. */
    public boolean isAimed() {
        if (currentState != AimState.TRACKING) return false;
        return Math.abs(lastAdjustedTx) < DEADBAND_DEG;
    }

    /** Returns true when the turret is in TRACKING state (camera sees hub tag). */
    public boolean isTracking() {
        return currentState == AimState.TRACKING;
    }

    /**
     * Distance to hub in meters from camera measurement.
     * Persists the last seen value after tag loss (stale but usable for mid-shot).
     */
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
