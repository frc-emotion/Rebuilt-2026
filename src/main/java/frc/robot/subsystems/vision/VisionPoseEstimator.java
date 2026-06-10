package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Minimal single-camera pose estimation for the turret-mounted camera (team requirement D11, "as
 * simple as possible"). The documented photonlib pattern plus three gates:
 *
 * <ol>
 *   <li>Turret slew gate — skip frames while the turret moves fast, so the CURRENT turret angle is
 *       a valid stand-in for the angle at capture time (no angle-history buffering).
 *   <li>Ambiguity gate — single-tag frames above MAX_POSE_AMBIGUITY are dropped.
 *   <li>On-field gate — estimates outside the field rectangle are dropped.
 * </ol>
 *
 * Refuses to run until the two mounting transforms are actually measured
 * (VisionConstants.kTransformsMeasured) — a guessed camera transform poisons odometry.
 */
public class VisionPoseEstimator {

  /** Sink for accepted measurements — wired to Drive::addVisionMeasurement. */
  @FunctionalInterface
  public interface PoseMeasurementConsumer {
    void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }

  private final AprilTagFieldLayout fieldLayout;
  private final PhotonPoseEstimator estimator;
  private final PoseMeasurementConsumer measurementConsumer;

  private int acceptedCount = 0;
  private int rejectedCount = 0;

  public VisionPoseEstimator(AprilTagFieldLayout fieldLayout, PoseMeasurementConsumer consumer) {
    this.fieldLayout = fieldLayout;
    this.measurementConsumer = consumer;
    this.estimator = new PhotonPoseEstimator(fieldLayout, robotToCamera(0.0));
  }

  /** Feed one loop's frames. Turret angle/velocity come from the turret's rotor sensor. */
  public void process(
      List<PhotonPipelineResult> results, double turretAngleRot, double turretVelocityRps) {
    if (!VisionConstants.kTransformsMeasured) {
      return;
    }
    if (Math.abs(turretVelocityRps) > VisionConstants.kMaxTurretSlewForPoseRps) {
      rejectedCount += results.size();
      return;
    }
    estimator.setRobotToCameraTransform(robotToCamera(turretAngleRot));

    for (PhotonPipelineResult result : results) {
      // Photonlib-recommended two-call pattern: multi-tag first, single-tag fallback.
      Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
      boolean multiTag = estimate.isPresent();
      if (estimate.isEmpty()) {
        estimate = estimator.estimateLowestAmbiguityPose(result);
      }
      if (estimate.isEmpty()) {
        continue;
      }

      EstimatedRobotPose est = estimate.get();
      if (!multiTag && !singleTagAcceptable(est)) {
        rejectedCount++;
        continue;
      }
      Pose2d pose = est.estimatedPose.toPose2d();
      if (!onField(pose)) {
        rejectedCount++;
        continue;
      }

      measurementConsumer.accept(pose, est.timestampSeconds, stdDevsFor(est, multiTag));
      acceptedCount++;
    }
  }

  private boolean singleTagAcceptable(EstimatedRobotPose est) {
    if (est.targetsUsed.isEmpty()) {
      return false;
    }
    return est.targetsUsed.get(0).getPoseAmbiguity() <= VisionConstants.MAX_POSE_AMBIGUITY;
  }

  private boolean onField(Pose2d pose) {
    return pose.getX() >= 0.0
        && pose.getX() <= fieldLayout.getFieldLength()
        && pose.getY() >= 0.0
        && pose.getY() <= fieldLayout.getFieldWidth();
  }

  // Standard photonlib scheme: two fixed tiers scaled by average tag distance.
  private Matrix<N3, N1> stdDevsFor(EstimatedRobotPose est, boolean multiTag) {
    double totalDistance = 0.0;
    for (PhotonTrackedTarget target : est.targetsUsed) {
      totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    double avgDistance = totalDistance / Math.max(1, est.targetsUsed.size());
    double scale = 1.0 + (avgDistance * avgDistance / 30.0);
    Matrix<N3, N1> base =
        multiTag ? VisionConstants.kMultiTagStdDevs : VisionConstants.kSingleTagStdDevs;
    return base.times(scale);
  }

  public int getAcceptedCount() {
    return acceptedCount;
  }

  public int getRejectedCount() {
    return rejectedCount;
  }

  /** robotToCamera = robotToTurret × rotate(turret angle) × turretToCamera — one compose line. */
  static Transform3d robotToCamera(double turretAngleRot) {
    return VisionConstants.ROBOT_TO_TURRET
        .plus(
            new Transform3d(
                new Translation3d(),
                new Rotation3d(0, 0, Units.rotationsToRadians(turretAngleRot))))
        .plus(VisionConstants.TURRET_TO_CAMERA);
  }
}
