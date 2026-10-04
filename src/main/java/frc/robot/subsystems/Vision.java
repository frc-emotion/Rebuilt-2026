package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    @FunctionalInterface
    public interface EstimateConsumer {
        void accept(Pose2d pose, double timestamp);
    }
}
