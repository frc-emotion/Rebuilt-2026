package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Vision subsystem: single turret-mounted camera, two jobs.
 *
 * <p>1. TARGETING (ported verbatim from legacy): every cycle computes distanceToHub and yawToHubDeg
 * via camera→hub = camera→tag (PhotonVision) + tag→hub (fixed constant), with sticky tag tracking,
 * hub-before-passing classification, and stale-distance hold.
 *
 * <p>2. POSE ESTIMATION (new, D11): frames are also fed to VisionPoseEstimator, which pushes
 * accepted measurements into the drivetrain. Wiring happens in RobotContainer via suppliers — this
 * subsystem holds no references to other subsystems.
 */
@Logged
public class Vision extends SubsystemBase {
  private final VisionIO io;
  private VisionIOInputs inputs = VisionIOInputs.kEmpty;

  private PhotonPipelineResult latestResult;
  private boolean freshThisCycle = false;
  private double resultTimestamp = 0.0;

  // Stale-data hold: distance keeps its last good value when no tag is visible (W21)
  private double lastGoodDistance = 0.0;
  private double lastGoodPassingDistance = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean cameraConnected = false;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean seeingHubTag = false;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double distanceToHub = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double yawToHubDeg = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private int trackedTagId = -1;

  @Logged(importance = Logged.Importance.DEBUG)
  private double latencyMs = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double ambiguity = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double rawDeg = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean seeingPassingTag = false;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double distanceToPassingTag = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double yawToPassingTagDeg = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private int trackedPassingTagId = -1;

  private final VisionPoseEstimator poseEstimator;
  private final java.util.function.DoubleSupplier turretAngleRotSupplier;
  private final java.util.function.DoubleSupplier turretVelocityRpsSupplier;

  /**
   * Suppliers (not subsystem references) carry the turret angle/velocity in, and the pose estimator
   * carries accepted measurements out to the drivetrain — wired in RobotContainer.
   */
  public Vision(
      VisionIO io,
      VisionPoseEstimator poseEstimator,
      java.util.function.DoubleSupplier turretAngleRotSupplier,
      java.util.function.DoubleSupplier turretVelocityRpsSupplier) {
    this.io = io;
    this.poseEstimator = poseEstimator;
    this.turretAngleRotSupplier = turretAngleRotSupplier;
    this.turretVelocityRpsSupplier = turretVelocityRpsSupplier;
  }

  @Override
  public void periodic() {
    inputs = io.updateInputs();

    poseEstimator.process(
        inputs.unreadResults(),
        turretAngleRotSupplier.getAsDouble(),
        turretVelocityRpsSupplier.getAsDouble());

    // 1. Drain all unread frames, keep the newest one
    freshThisCycle = false;
    seeingHubTag = false; // reset both flags every cycle up here (legacy FIX 3)
    seeingPassingTag = false;
    for (PhotonPipelineResult result : inputs.unreadResults()) {
      latestResult = result;
      freshThisCycle = true;
      resultTimestamp = result.getTimestampSeconds();
    }

    cameraConnected = inputs.connected();

    // 2. Nothing new or no targets? Hold stale distance, clear tracking flag
    if (!freshThisCycle || latestResult == null || !latestResult.hasTargets()) {
      distanceToHub = lastGoodDistance;
      distanceToPassingTag = lastGoodPassingDistance;
      return;
    }

    // 3. Find target — prefer the tag we're already tracking (sticky, W22)
    PhotonTrackedTarget bestTarget = null;
    double bestAmbiguity = 1.0;

    if (trackedTagId != -1) {
      for (PhotonTrackedTarget target : latestResult.getTargets()) {
        if (target.getFiducialId() != trackedTagId) {
          continue;
        }
        double amb = target.getPoseAmbiguity();
        if (!VisionConstants.BENCH_TEST_ANY_TAG && amb > VisionConstants.MAX_POSE_AMBIGUITY) {
          break;
        }
        bestTarget = target;
        bestAmbiguity = amb;
        break;
      }
    }

    // Second pass: current tag not found — pick the best new one
    if (bestTarget == null) {
      for (PhotonTrackedTarget target : latestResult.getTargets()) {
        int id = target.getFiducialId();
        if (!VisionConstants.BENCH_TEST_ANY_TAG
            && !VisionConstants.isOurHubTag(id)
            && !VisionConstants.isOurPassingTag(id)) {
          continue;
        }
        double amb = target.getPoseAmbiguity();
        if (!VisionConstants.BENCH_TEST_ANY_TAG && amb > VisionConstants.MAX_POSE_AMBIGUITY) {
          continue;
        }
        if (amb < bestAmbiguity) {
          bestTarget = target;
          bestAmbiguity = amb;
        }
      }
    }

    if (bestTarget == null) {
      distanceToHub = lastGoodDistance;
      distanceToPassingTag = lastGoodPassingDistance;
      return;
    }

    int tagId = bestTarget.getFiducialId();

    // Hub checked BEFORE passing: the ID sets overlap and this precedence is load-bearing (W28).
    if (VisionConstants.isOurHubTag(tagId)) {
      seeingHubTag = true;
    } else if (VisionConstants.isOurPassingTag(tagId)) {
      seeingPassingTag = true;
    }

    // 4. Compute distance + yaw
    if (VisionConstants.BENCH_TEST_ANY_TAG) {
      seeingHubTag = true; // bench mode treats any tag as a hub tag
      yawToHubDeg = bestTarget.getYaw();
      Translation3d camToTag = bestTarget.getBestCameraToTarget().getTranslation();
      distanceToHub = Math.hypot(camToTag.getX(), camToTag.getY());
    } else if (seeingHubTag) {
      rawDeg = bestTarget.getYaw();
      Transform3d cameraToTag = bestTarget.getBestCameraToTarget();
      Transform3d tagToHub = VisionConstants.TAG_TO_HUB_CENTER.get(tagId);

      Translation3d toHub =
          (tagToHub != null)
              ? cameraToTag.plus(tagToHub).getTranslation()
              : cameraToTag.getTranslation();

      distanceToHub = Math.hypot(toHub.getX(), toHub.getY());
      // Negated so positive = "target to the right", matching PhotonVision's screen convention
      // and keeping both code paths sign-consistent for the turret controller (W26).
      yawToHubDeg = -Math.toDegrees(Math.atan2(toHub.getY(), toHub.getX()));
    } else if (seeingPassingTag) {
      rawDeg = bestTarget.getYaw();
      Transform3d cameraToTag = bestTarget.getBestCameraToTarget();

      distanceToPassingTag =
          Math.hypot(cameraToTag.getTranslation().getX(), cameraToTag.getTranslation().getY());

      // Aim along the tag's outward surface NORMAL (lob the pass into the zone the tag faces),
      // not at the tag itself (W25).
      Rotation3d tagToCamRot = cameraToTag.getRotation().unaryMinus();
      Translation3d tagNormalInCam = new Translation3d(0, 0, 1).rotateBy(tagToCamRot);
      yawToPassingTagDeg =
          -Math.toDegrees(Math.atan2(-tagNormalInCam.getY(), -tagNormalInCam.getX()));
    }

    // 5. Update state
    trackedTagId = tagId;
    ambiguity = bestAmbiguity;
    latencyMs = latestResult.metadata.getLatencyMillis();

    if (seeingHubTag) {
      lastGoodDistance = distanceToHub;
    }
    if (seeingPassingTag) {
      trackedPassingTagId = tagId;
      lastGoodPassingDistance = distanceToPassingTag;
    }
  }

  // ── Public API (legacy surface, verbatim) ──

  public double getDistanceToHub() {
    return distanceToHub;
  }

  public double getYawToHubDeg() {
    return yawToHubDeg;
  }

  public boolean isSeeingHubTag() {
    return seeingHubTag;
  }

  public boolean isResultFresh() {
    return freshThisCycle;
  }

  public double getResultTimestamp() {
    return resultTimestamp;
  }

  public int getTrackedTagId() {
    return trackedTagId;
  }

  public boolean isSeeingPassingTag() {
    return seeingPassingTag;
  }

  public double getDistanceToPassingTag() {
    return distanceToPassingTag;
  }

  public double getYawToPassingTagDeg() {
    return yawToPassingTagDeg;
  }

  public int getTrackedPassingTagId() {
    return trackedPassingTagId;
  }
}
