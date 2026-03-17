package frc.robot.commands;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.epilogue.Logged;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * Test command: turret tracks hub AprilTags using direct voltage control.
 * Voltage is proportional to camera tx error (standard FRC turret tracking).
 * When no tag is visible, holds position via MotionMagic.
 * Includes soft-limit safety near turret range edges.
 */
@Logged
public class TurretVisionTrackingTest extends Command {

    private final Vision vision;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;

    @Logged private double visionTxDeg = 0.0;
    @Logged private double commandedRPS = 0.0;
    @Logged private double actualVelocityRPS = 0.0;
    @Logged private double motorVoltage = 0.0;
    @Logged private double turretPositionRot = 0.0;
    @Logged private boolean tracking = false;
    @Logged private int trackedTagId = -1;

    // RPS per degree of camera tx error. Turret spins at this rate to center the tag.
    private static final double kTxToRPS = 0.06; // 10 deg off = 0.6 RPS
    private static final double MAX_TRACKING_RPS = 1.5; // cap turret speed
    private static final double DEADBAND_DEG = 1.0; // don't correct below this
    private static final double TEST_SHOOTER_RPS = 20.0;

    // VelocityVoltage uses Slot1 (velocity gains) instead of Slot0 (position/MotionMagic gains)
    private final VelocityVoltage trackingVelocityRequest = new VelocityVoltage(0).withSlot(1);
    private double lastVisionTimestamp = 0.0;
    private double holdPositionRot = 0.0;
    private double lastDesiredRPS = 0.0;
    private double lastTrackingTimeSec = 0.0;
    private static final double TARGET_LOST_TIMEOUT_SEC = 0.25; // hold velocity for 250ms after last frame

    public TurretVisionTrackingTest(Vision vision, Turret turret, Hood hood, Shooter shooter) {
        this.vision = vision;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        addRequirements(turret, hood, shooter);
    }

    @Override
    public void initialize() {
        holdPositionRot = turret.getTurretMotor().getPosition().getValueAsDouble();
        lastVisionTimestamp = 0.0;
        tracking = false;
        trackedTagId = -1;
    }

    @Override
    public void execute() {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // Process fresh vision frames when available
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
                        trackedTagId = tagId;
                        lastTrackingTimeSec = now;

                        // Compute new velocity from fresh tx
                        if (Math.abs(tx) > DEADBAND_DEG) {
                            lastDesiredRPS = MathUtil.clamp(kTxToRPS * tx, -MAX_TRACKING_RPS, MAX_TRACKING_RPS);
                        } else {
                            lastDesiredRPS = 0.0;
                        }
                    }
                }
            }
        }

        // Determine if we're still actively tracking (within timeout of last fresh frame)
        tracking = (now - lastTrackingTimeSec) < TARGET_LOST_TIMEOUT_SEC;

        if (tracking) {
            double desiredRPS = lastDesiredRPS;

            // Soft-limit safety: don't spin toward a limit we're already near
            double pos = turret.getTurretMotor().getPosition().getValueAsDouble();
            double margin = 0.03;
            if (pos > TurretConstants.TURRET_FORWARD_LIMIT - margin && desiredRPS > 0) {
                desiredRPS = 0;
            }
            if (pos < TurretConstants.TURRET_REVERSE_LIMIT + margin && desiredRPS < 0) {
                desiredRPS = 0;
            }

            commandedRPS = desiredRPS;
            turret.getTurretMotor().setControl(trackingVelocityRequest.withVelocity(desiredRPS));
            holdPositionRot = pos;
        } else {
            // Target lost: hold position with MotionMagic (Slot0)
            commandedRPS = 0.0;
            turret.moveTurret(Rotations.of(holdPositionRot));
        }

        // Update telemetry every cycle regardless of tracking state
        actualVelocityRPS = turret.getTurretMotor().getVelocity().getValueAsDouble();
        motorVoltage = turret.getTurretMotor().getMotorVoltage().getValueAsDouble();
        turretPositionRot = turret.getTurretMotor().getPosition().getValueAsDouble();

        // Keep shooter spinning at test speed
        shooter.setShooterSpeed(RotationsPerSecond.of(TEST_SHOOTER_RPS));
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
