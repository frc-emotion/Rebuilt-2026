package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * Test command: turret tracks hub AprilTags using vision only (no odometry,
 * no hood/shooter). Includes wrap-around snap — when the turret hits a range
 * limit, it snaps to the opposite end to continue tracking.
 *
 * <p>Bind to a button with .whileTrue() for testing. Only requires the turret
 * subsystem so hood/shooter remain free.
 *
 * <p>Uses the turret camera's best visible hub tag yaw (tx) with a P-controller
 * to steer the turret. When no tag is visible, holds the last setpoint.
 */
@Logged
public class TurretVisionTrackingTest extends Command {

    private final Vision vision;
    private final Turret turret;

    @Logged private double encoderSetpointRot = 0.0;
    @Logged private double visionTxDeg = 0.0;
    @Logged private boolean tracking = false;
    @Logged private int trackedTagId = -1;

    private static final double kVisionP = 0.003;

    private double lastVisionTimestamp = 0.0;

    public TurretVisionTrackingTest(Vision vision, Turret turret) {
        this.vision = vision;
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        encoderSetpointRot = turret.getTurretMotor().getPosition().getValueAsDouble();
        lastVisionTimestamp = 0.0;
        tracking = false;
        trackedTagId = -1;
    }

    @Override
    public void execute() {
        tracking = false;
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

                    // Only track hub tags (or any tag if bench testing)
                    if (VisionConstants.BENCH_TEST_ANY_TAG || VisionConstants.isHubTag(tagId)) {
                        double tx = target.get().getYaw();
                        visionTxDeg = tx;
                        tracking = true;
                        trackedTagId = tagId;

                        double currentPos = turret.getTurretMotor().getPosition().getValueAsDouble();
                        encoderSetpointRot = currentPos + (kVisionP * tx) + TurretConstants.TURRET_AIM_OFFSET;
                    }
                }
            }
        }

        // Use wrap-around: if setpoint exceeds range, snap to opposite end
        turret.moveTurretWithWrap(Rotations.of(encoderSetpointRot), 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
