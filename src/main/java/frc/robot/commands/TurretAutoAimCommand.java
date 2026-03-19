package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.util.TurretAimingCalculator;

import static edu.wpi.first.units.Units.Rotations;

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
    @Logged(importance = Logged.Importance.DEBUG) private double targetPositionRot = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double currentPositionRot = 0.0;

    // ================================================================
    //  TUNING CONSTANTS
    // ================================================================

    // Convert camera tx (degrees) to turret rotations.
    // 1 degree of tx ≈ 1/360 turret rotations, but we scale by a gain
    // to let the turret converge faster or slower. Start at 1.0 (1:1).
    // If the turret under-shoots, increase. If it oscillates, decrease.
    private static final double TX_TO_ROT_GAIN = 1.0;

    // Don't correct below this (prevents chasing noise near center)
    private static final double DEADBAND_DEG = 1.0;

    // TRACKING → MANUAL: persist tracking this long after last fresh frame.
    private static final double TRACKING_PERSIST_SEC = 0.15;

    // Manual joystick deadband
    private static final double MANUAL_DEADBAND = 0.08;

    // ================================================================
    //  INTERNAL STATE
    // ================================================================
    private AimState currentState = AimState.MANUAL;
    private double lastVisionTimestamp = 0.0;
    private double lastTagSeenTimeSec = 0.0;
    private double lastCameraDistance = 0.0;

    // ================================================================
    //  CONSTRUCTOR
    // ================================================================
    /**
     * Creates a unified turret command.
     *
     * @param drivetrain        swerve drivetrain (unused for now, kept for future gyro FF)
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
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        // ============================================================
        //  Step 1: Read fresh vision data (tx yaw + distance)
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

        // Distance for interp tables (hood/shooter)
        distanceToHubMeters = lastCameraDistance;

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
                targetPositionRot = turretPos;
                break;
            }

            case TRACKING: {
                // Convert camera tx to a turret position offset.
                // PhotonVision tx: positive = target is left of camera center.
                // Motor: Clockwise_Positive → positive rotation = CW (looking down).
                // Target to the LEFT → turret must rotate LEFT (CCW) → negative offset.
                // So: offset = -(tx / 360)
                double txOffsetRot = 0.0;
                if (Math.abs(visionTxDeg) > DEADBAND_DEG) {
                    txOffsetRot = -(visionTxDeg / 360.0) * TX_TO_ROT_GAIN;
                }

                double desiredPos = turretPos + txOffsetRot;
                targetPositionRot = desiredPos;

                // MotionMagic handles acceleration, deceleration, and holding.
                // Slot 0 already has tuned P/I gains. Soft limits in config
                // prevent exceeding range.
                turret.moveTurret(Rotations.of(desiredPos));
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
    /** Returns true when TRACKING and tx is within deadband. */
    public boolean isAimed() {
        if (currentState != AimState.TRACKING) return false;
        return Math.abs(visionTxDeg) < DEADBAND_DEG;
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
