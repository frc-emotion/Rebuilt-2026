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
 * Turret command: 2-state machine (MANUAL / TRACKING).
 *
 * MANUAL: operator joystick → voltage control.
 * TRACKING: camera tx → MotionMagic position correction each cycle.
 *
 * Transitions:
 *   MANUAL → TRACKING: instant when a hub tag is detected.
 *   TRACKING → MANUAL: after TRACKING_PERSIST_SEC of no tag.
 */
@Logged
public class TurretAutoAimCommand extends Command {

    public enum AimState { MANUAL, TRACKING }

    // ── Subsystems & inputs ──
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final Turret turret;
    private final TurretAimingCalculator calculator;
    private final DoubleSupplier joystickSupplier;

    // ── Telemetry ──
    @Logged(importance = Logged.Importance.CRITICAL) private String state = "MANUAL";
    @Logged(importance = Logged.Importance.CRITICAL) private double distanceToHubMeters = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double visionTxDeg = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean visionActive = false;
    @Logged(importance = Logged.Importance.CRITICAL) private int trackedTagId = -1;
    @Logged(importance = Logged.Importance.CRITICAL) private double targetPositionRot = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double currentPositionRot = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double turretErrorRot = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double gyroFeedforwardRot = 0.0;

    // ── Tuning constants ──
    private static final double TX_TO_ROT_GAIN = 0.5;
    private static final double DEADBAND_DEG = 2.0;
    private static final double TRACKING_PERSIST_SEC = 0.15;
    private static final double MANUAL_DEADBAND = 0.08;
    private static final boolean BENCH_TEST_BYPASS_AIM_GATE = false;
    private static final double LOOP_PERIOD_SEC = 0.02;

    // ⚠️ CHECK SIGN ON ROBOT FIRST — if turret drifts opposite to robot rotation, negate the sign below.
    private static final boolean GYRO_FF_ENABLED = true;

    // ── Internal state ──
    private AimState currentState = AimState.MANUAL;
    private double lastVisionTimestamp = 0.0;
    private double lastTagSeenTimeSec = 0.0;
    private double lastCameraDistance = 0.0;
    private double persistentTargetRot = 0.0;
    private boolean freshVisionThisCycle = false;

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

        // Step 1: Read fresh vision data
        freshVisionThisCycle = false;
        if (vision != null && vision.isTurretResultFresh() && vision.turretCameraHasTargets()) {
            double ts = vision.getTurretResultTimestamp();
            if (ts != lastVisionTimestamp) {
                lastVisionTimestamp = ts;
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

        distanceToHubMeters = lastCameraDistance;

        // Step 2: Is the tag still fresh?
        boolean tagFresh = lastTagSeenTimeSec > 0 && (now - lastTagSeenTimeSec) < TRACKING_PERSIST_SEC;
        visionActive = tagFresh;

        // Step 3: State transitions
        if (currentState == AimState.MANUAL && tagFresh) {
            currentState = AimState.TRACKING;
        } else if (currentState == AimState.TRACKING && !tagFresh) {
            currentState = AimState.MANUAL;
        }
        state = currentState.name();

        // Step 4: Execute current state
        double turretPos = turret.getTurretMotor().getPosition().getValueAsDouble();
        currentPositionRot = turretPos;

        if (currentState == AimState.MANUAL) {
            double input = MathUtil.applyDeadband(joystickSupplier.getAsDouble(), MANUAL_DEADBAND);
            turret.setTurretVoltage(MathUtil.clamp(input, -1, 1));
            persistentTargetRot = turretPos;
            targetPositionRot = turretPos;
        } else {
            // Gyro FF: counter-rotate for robot yaw every cycle
            gyroFeedforwardRot = 0.0;
            if (GYRO_FF_ENABLED && drivetrain != null) {
                double gyroRateDegPerSec = drivetrain.getPigeon2()
                        .getAngularVelocityZWorld().getValueAsDouble();
                gyroFeedforwardRot = -(gyroRateDegPerSec / 360.0) * LOOP_PERIOD_SEC;
                persistentTargetRot += gyroFeedforwardRot;
            }

            // Vision correction: only on fresh frames, outside deadband
            if (freshVisionThisCycle && Math.abs(visionTxDeg) > DEADBAND_DEG) {
                persistentTargetRot += (visionTxDeg / 360.0) * TX_TO_ROT_GAIN;
            }

            persistentTargetRot = MathUtil.clamp(persistentTargetRot,
                    frc.robot.Constants.TurretConstants.TURRET_REVERSE_LIMIT,
                    frc.robot.Constants.TurretConstants.TURRET_FORWARD_LIMIT);

            targetPositionRot = persistentTargetRot;
            turret.moveTurret(Rotations.of(persistentTargetRot));
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

    // ── Public accessors ──

    public boolean isAimed() {
        if (BENCH_TEST_BYPASS_AIM_GATE) return true;
        if (currentState != AimState.TRACKING) return false;
        return Math.abs(visionTxDeg) < DEADBAND_DEG;
    }

    public boolean isTracking() {
        return currentState == AimState.TRACKING;
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
