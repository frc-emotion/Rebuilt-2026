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
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

@Logged
public class Vision extends SubsystemBase {
    @NotLogged public final PhotonCamera shootCam;
    @NotLogged public final PhotonPoseEstimator photonEstimator;
    @NotLogged public final EstimateConsumer estConsumer;

    // Initialized so nothing is null before DriverStation / odometry data arrives
    @NotLogged private Optional<Alliance> allianceColor = Optional.empty();
    private Pose2d currentPose = new Pose2d();

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

    public void setAllianceColor(Optional<Alliance> alliance) {
        allianceColor = (alliance == null) ? Optional.empty() : alliance;
    }

    public void setCurrentPose(Pose2d pose) {
        if (pose != null) {
            currentPose = pose;
        }
    }

    /**
     * Robot-relative angle the turret needs to point at the alliance hub,
     * wrapped to [-pi, pi]. Empty if the alliance isn't known yet.
     */
    public Optional<Angle> getTurretToHub() {
        if (allianceColor.isEmpty()) {
            return Optional.empty();
        }

        Translation2d hub = (allianceColor.get() == Alliance.Blue)
                ? VisionConstants.kBlueHubCenter.getTranslation()
                : VisionConstants.kRedHubCenter.getTranslation();

        // Field-relative direction from robot to hub
        Rotation2d fieldAngle = hub.minus(currentPose.getTranslation()).getAngle();

        // Convert to robot-relative; Rotation2d.minus wraps to [-pi, pi]
        Rotation2d turretAngle = fieldAngle.minus(currentPose.getRotation());

        return Optional.of(Radians.of(turretAngle.getRadians()));
    }
}