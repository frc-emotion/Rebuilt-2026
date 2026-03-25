package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.util.TurretAimingCalculator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Degrees;

/**
 * Simplified turret command with 2-state machine:
 *
 * <p><b>MANUAL</b> — No hub tag visible. Operator controls turret via joystick.
 *
 * <p><b>TRACKING</b> — Turret camera sees a hub tag. Each cycle, the camera's
 * tx (yaw error) is converted to a turret rotation offset and added to the
 * current turret position. The result is commanded via MotionMagic (Slot 0),
 * which handles smooth acceleration, deceleration, and position holding.
 *
 * <p>Transition MANUAL → TRACKING: instant when a hub tag is detected.
 * Transition TRACKING → MANUAL: after 0.15s of no tag (prevents single-frame drops).
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
    // ================================================================
    @Logged(importance = Logged.Importance.CRITICAL) private String state = "MANUAL";
    @Logged(importance = Logged.Importance.CRITICAL) private double distanceToHubMeters = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double visionTxDeg = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean visionActive = false;
    @Logged(importance = Logged.Importance.CRITICAL) private int trackedTagId = -1;
    @Logged(importance = Logged.Importance.CRITICAL) private double targetPositionRot = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double currentPositionRot = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double turretErrorRot = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double gyroFeedforwardRot = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double leadAngleRot = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double effectiveDistanceMeters = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double shooterRPSOffset = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double robotSpeedMPS = 0.0;

    // ================================================================
    //  TUNING CONSTANTS
    // ================================================================

    // Convert camera tx (degrees) to turret rotations.
    // 1 degree of tx ≈ 1/360 turret rotations, but we scale by a gain
    // to let the turret converge faster or slower. Start at 1.0 (1:1).
    // If the turret under-shoots, increase. If it oscillates, decrease.
    private static final double TX_TO_ROT_GAIN = 1.0;

    // Don't correct below this (prevents chasing noise near center)
    private static final double DEADBAND_DEG = 6.7;

    // TRACKING → MANUAL: persist tracking this long after last fresh frame.
    private static final double TRACKING_PERSIST_SEC = 0.15;

    // Manual joystick deadband
    private static final double MANUAL_DEADBAND = 0.08;

    // Bench test: bypass aim gate so indexers fire even when not tracking/aimed.
    // SET TO FALSE FOR COMPETITION.
    private static final boolean BENCH_TEST_BYPASS_AIM_GATE = false;

    // Gyro feedforward: when the robot rotates, pre-compensate the turret
    // so vision only needs to correct the residual error.
    // Set to false to disable during initial testing.
    private static final boolean GYRO_FF_ENABLED = false;
    private static final double LOOP_PERIOD_SEC = 0.02; // 50 Hz main loop

    // ── Shoot-on-the-move (SOTM) constants ──
    // Master enable: set false to revert to stationary-only aiming
    private static final boolean SOTM_ENABLED = false;
    // Flywheel wheel radius for ball exit speed estimation
    private static final double FLYWHEEL_RADIUS_METERS = edu.wpi.first.math.util.Units.inchesToMeters(2.0);
    // Fraction of flywheel surface speed transferred to the ball
    private static final double BALL_EXIT_EFFICIENCY = 0.7;
    // Disable SOTM compensation above this robot speed (m/s) — too unreliable
    private static final double MAX_SOTM_SPEED_MPS = 2.0;
    // Relaxed aim tolerance for moving shots (degrees)
    private static final double SOTM_AIM_TOLERANCE_DEG = 3.0;
    // Estimated vision pipeline latency for latency compensation (seconds)
    private static final double VISION_LATENCY_SEC = 0.05;

    // ================================================================
    //  INTERNAL STATE
    // ================================================================
    private AimState currentState = AimState.MANUAL;
    private double lastVisionTimestamp = 0.0;
    private double lastTagSeenTimeSec = 0.0;
    private double lastCameraDistance = 0.0;
    private double persistentTargetRot = 0.0;
    private boolean freshVisionThisCycle = false;

    // ================================================================
    //  CONSTRUCTOR
    // ================================================================
    /**
     * Creates a unified turret command.
     *
     * @param drivetrain        swerve drivetrain (provides gyro yaw rate for feedforward)
     * @param vision            vision subsystem (turret camera)
     * @param turret            turret subsystem
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
        lastCameraDistance = 0.0;
        visionTxDeg = 0.0;
        persistentTargetRot = turret.getTurretMotor().getPosition().getValueAsDouble();
        freshVisionThisCycle = false;
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        // ============================================================
        //  Step 1: Read fresh vision data (tx yaw + distance)
        // ============================================================
        freshVisionThisCycle = false;
        if (vision != null
                && vision.isTurretResultFresh()
                && vision.turretCameraHasTargets()) {
            double currentTimestamp = vision.getTurretResultTimestamp();
            if (currentTimestamp != lastVisionTimestamp) {
                lastVisionTimestamp = currentTimestamp;

                var target = vision.getTurretCameraBestTarget();
                if (target.isPresent()) {
                    int tagId = target.get().getFiducialId();

                    if (VisionConstants.BENCH_TEST_ANY_TAG || VisionConstants.isDSFacingHubTag(tagId)) {
                        visionTxDeg = target.get().getYaw();
                        trackedTagId = tagId;
                        lastTagSeenTimeSec = now;
                        lastCameraDistance = vision.getTurretCameraDistanceToTarget();
                        freshVisionThisCycle = true;
                    }
                }
            }
        }

        // Distance for interp tables (hood/shooter)
        distanceToHubMeters = lastCameraDistance;
        // Update effective distance unconditionally so ShootCommand always
        // has a valid value, even when turret is in MANUAL state.
        effectiveDistanceMeters = distanceToHubMeters;

        // ============================================================
        //  Step 2: Determine if tag is fresh
        // ============================================================
        double timeSinceTag = now - lastTagSeenTimeSec;
        boolean tagFresh = lastTagSeenTimeSec > 0 && timeSinceTag < TRACKING_PERSIST_SEC;
        visionActive = tagFresh;

        // ============================================================
        //  Step 3: State transitions
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
        }
        state = currentState.name();

        // ============================================================
        //  Step 4: State execution
        // ============================================================
        double turretPos = turret.getTurretMotor().getPosition().getValueAsDouble();
        currentPositionRot = turretPos;

        switch (currentState) {
            case MANUAL: {
                double input = MathUtil.applyDeadband(joystickSupplier.getAsDouble(), MANUAL_DEADBAND);
                double clampedInput = MathUtil.clamp(input, -1, 1);
                turret.setTurretVoltage(clampedInput);
                // Keep persistent target in sync so TRACKING starts from current pos
                persistentTargetRot = turretPos;
                targetPositionRot = turretPos;
                break;
            }

            case TRACKING: {
                // --- Gyro feedforward: counter-rotate for robot yaw ---
                // Applied every cycle (not just on fresh vision frames) so the
                // turret tracks even between camera frames.
                gyroFeedforwardRot = 0.0;
                if (GYRO_FF_ENABLED && drivetrain != null) {
                    // getAngularVelocityZWorld returns deg/s; positive = CCW
                    // Robot rotating CCW means turret must rotate CW (positive
                    // in Clockwise_Positive motor convention) to stay aimed.
                    double gyroRateDegPerSec = drivetrain.getPigeon2()
                            .getAngularVelocityZWorld().getValueAsDouble();
                    gyroFeedforwardRot = -(gyroRateDegPerSec / 360.0) * LOOP_PERIOD_SEC;
                    persistentTargetRot += gyroFeedforwardRot;
                }

                // --- Vision correction: only on fresh frames ---
                // persistentTargetRot accumulates corrections — it is NOT
                // re-read from the motor each cycle, preventing the base
                // from shifting while the motor is still catching up.
                Angle newSetpoint = Rotations.of(turretPos);
                if (freshVisionThisCycle && Math.abs(visionTxDeg) > DEADBAND_DEG) {
                    double txOffsetRot = -(visionTxDeg / 360.0) * TX_TO_ROT_GAIN;
                    persistentTargetRot += txOffsetRot;
                    newSetpoint = newSetpoint.plus(Degrees.of(visionTxDeg));
                }

                // --- Shoot-on-the-move: effective distance + shooter compensation ---
                // Lead angle is NOT applied to the turret here (SOTM_ENABLED = false).
                // When enabled, lead will be added to newSetpoint in turret-frame degrees.
                leadAngleRot = 0.0;
                effectiveDistanceMeters = distanceToHubMeters;
                shooterRPSOffset = 0.0;
                robotSpeedMPS = 0.0;
                if (SOTM_ENABLED && drivetrain != null && distanceToHubMeters > 0.5) {
                    computeSOTMCompensation(turretPos);
                    // Add lead angle to newSetpoint in degrees (same frame as tx)
                    newSetpoint = newSetpoint.plus(Degrees.of(leadAngleRot * 360.0));
                }

                persistentTargetRot = MathUtil.clamp(persistentTargetRot,
                        frc.robot.Constants.TurretConstants.TURRET_REVERSE_LIMIT,
                        frc.robot.Constants.TurretConstants.TURRET_FORWARD_LIMIT);

                // MotionMagic handles acceleration, deceleration, and holding.
                // Commands every cycle so turret holds position between frames.
                targetPositionRot = turretPos;
                turret.moveTurret(newSetpoint);
                break;
            }
        }

        turretErrorRot = targetPositionRot - turretPos;
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
    /** Returns true when TRACKING and tx is within deadband, or always true in bench test mode. */
    public boolean isAimed() {
        if (BENCH_TEST_BYPASS_AIM_GATE) return true;
        if (currentState != AimState.TRACKING) return true; // should be false, but just in case, require tracking state for aiming
        double tolerance = (SOTM_ENABLED && robotSpeedMPS > 0.3)
                ? SOTM_AIM_TOLERANCE_DEG : DEADBAND_DEG;
        return Math.abs(visionTxDeg) < tolerance;
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

    /**
     * Effective distance to hub, compensated for robot radial velocity when SOTM is active.
     * Use this instead of getDistanceToHub() for interp table lookups in ShootCommand.
     */
    public double getEffectiveDistance() {
        return effectiveDistanceMeters;
    }

    /** Shooter RPS offset from SOTM. Add to base interp table value. */
    public double getShooterRPSOffset() {
        return shooterRPSOffset;
    }

    /** Whether SOTM compensation is currently active. */
    public boolean isSOTMActive() {
        return SOTM_ENABLED && robotSpeedMPS > 0.3 && currentState == AimState.TRACKING;
    }

    // ================================================================
    //  SOTM COMPUTATION
    // ================================================================
    /**
     * Computes lead angle, effective distance, and shooter speed offset
     * based on robot velocity relative to the hub.
     */
    private void computeSOTMCompensation(double turretPosRot) {
        // Get robot velocity in field frame
        var driveState = drivetrain.getState();
        if (driveState == null || driveState.Speeds == null || driveState.Pose == null) return;
        ChassisSpeeds robotSpeeds = driveState.Speeds;
        Rotation2d heading = driveState.Pose.getRotation();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, heading);

        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;
        robotSpeedMPS = Math.hypot(vx, vy);

        // Skip if robot is barely moving or too fast
        if (robotSpeedMPS < 0.3 || robotSpeedMPS > MAX_SOTM_SPEED_MPS) {
            return;
        }

        // Vector from robot to hub center (field frame)
        Translation2d hubCenter = calculator.getTargetHubCenter();
        if (hubCenter == null) return;
        Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
        Translation2d robotToHub = hubCenter.minus(robotPos);
        double dist = robotToHub.getNorm();
        if (dist < 0.5) return;

        // Unit vector toward hub
        Translation2d unitToHub = robotToHub.div(dist);

        // Decompose velocity into radial (toward hub) and tangential components
        double vRadial = vx * unitToHub.getX() + vy * unitToHub.getY();
        double vTangential = -vx * unitToHub.getY() + vy * unitToHub.getX();

        // Estimate ball flight time using camera distance (matches interp table calibration)
        double baseShooterRPS = calculator.getFlywheelRPS(distanceToHubMeters);
        double ballExitSpeedMPS = baseShooterRPS * 2.0 * Math.PI * FLYWHEEL_RADIUS_METERS * BALL_EXIT_EFFICIENCY;
        if (ballExitSpeedMPS < 1.0) return;
        double flightTimeSec = distanceToHubMeters / ballExitSpeedMPS;

        // Lead angle: aim ahead so ball arrives at hub
        double leadAngleRad = Math.atan2(vTangential * flightTimeSec, dist);
        // Convert to turret rotations (turret gear ratio handled by motor config)
        leadAngleRot = leadAngleRad / (2.0 * Math.PI);

        // Effective distance: use camera distance (matches interp table calibration)
        // and adjust for radial motion during flight
        effectiveDistanceMeters = distanceToHubMeters - vRadial * flightTimeSec;
        effectiveDistanceMeters = Math.max(effectiveDistanceMeters, 1.0);

        // Shooter speed offset: compensate for radial velocity
        // If approaching hub, ball needs less speed; if retreating, more
        double wheelCircumference = 2.0 * Math.PI * FLYWHEEL_RADIUS_METERS;
        shooterRPSOffset = -(vRadial / wheelCircumference) / BALL_EXIT_EFFICIENCY;
    }
}
