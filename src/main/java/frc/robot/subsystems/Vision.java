package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.VisionConstants;

@Logged
public class Vision extends SubsystemBase {
    @NotLogged private final PhotonCamera shootCam;
    @NotLogged private final PhotonPoseEstimator photonEstimator;
    @NotLogged private final EstimateConsumer estConsumer;

    public Vision(EstimateConsumer estConsumer) {
        shootCam = new PhotonCamera(VisionConstants.shootCam);
        photonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kRobotToCam);
        this.estConsumer = estConsumer;
    }

    public final void update() {
        for (var result : shootCam.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }

            // updateEstimationStdDevs(visionEst, result.getTargets());

            visionEst.ifPresent(
                    est -> estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds));
        }
    }

    public final double updateTurretSetpoint(Alliance alliance, Pose2d drivetrainPose){
        Translation2d hub = alliance == Alliance.Blue ? FieldConstants.BLUE_HUB : FieldConstants.RED_HUB;
        double hubBearingRot = bearingRot(drivetrainPose, hub);
        double turretSetpointRot = turretRotForBearing(hubBearingRot);
        return turretSetpointRot;

    }

        /** Angle from the intake to the target, counterclockwise-positive, in [-0.5, 0.5]. */
    public static double bearingRot(Pose2d robotPose, Translation2d target) {
        Rotation2d fieldAngleToTarget = target.minus(robotPose.getTranslation()).getAngle();
        return fieldAngleToTarget.minus(robotPose.getRotation()).getRotations();
    }

    /**
     * Turret position that points along a bearing. The turret counts clockwise from its boot
     * heading while bearings count counterclockwise, so the position is boot heading minus bearing.
     * Turret.setSetpoint wraps the result into the travel limits.
     */
    public static double turretRotForBearing(double bearingRot) {
        return TurretConstants.BOOT_HEADING_ROT - bearingRot;
    }

    /** Pose at the robot center facing the field direction the turret points: robot heading plus boot heading minus turret position. */
    public static Pose2d turretFieldPose(Pose2d robotPose, double turretRot) {
        Rotation2d turretRelativeToRobot = Rotation2d.fromRotations(TurretConstants.BOOT_HEADING_ROT - turretRot);
        return new Pose2d(robotPose.getTranslation(), robotPose.getRotation().plus(turretRelativeToRobot));
    }

    @FunctionalInterface
    public interface EstimateConsumer {
        void accept(Pose2d pose, double timestamp);
    }
}
