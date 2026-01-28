package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;

/**
 * Vision subsystem that manages multiple PhotonVision cameras for AprilTag
 * detection
 * and robot pose estimation.
 *
 * Based on PhotonVision documentation:
 * https://docs.photonvision.org/en/v2026.0.1-beta/docs/programming/photonlib/robot-pose-estimator.html
 */
public class Vision extends SubsystemBase {

    // Telemetry throttling - update SmartDashboard every N cycles (10 = 5Hz at 50Hz
    // loop)
    private static final int TELEMETRY_UPDATE_INTERVAL = 10;

    // Cameras
    private final PhotonCamera cameraRight;
    private final PhotonCamera cameraLeft;

    // Pose estimators for each camera
    private final PhotonPoseEstimator poseEstimatorRight;
    private final PhotonPoseEstimator poseEstimatorLeft;

    // Latest results cache
    private PhotonPipelineResult latestResultRight;
    private PhotonPipelineResult latestResultLeft;

    // Latest estimated poses
    private Optional<EstimatedRobotPose> latestEstimateRight = Optional.empty();
    private Optional<EstimatedRobotPose> latestEstimateLeft = Optional.empty();

    // Standard deviations for the latest estimates
    private Matrix<N3, N1> currentStdDevsRight = VisionConstants.SINGLE_TAG_STD_DEVS;
    private Matrix<N3, N1> currentStdDevsLeft = VisionConstants.SINGLE_TAG_STD_DEVS;

    // Diagnostic counters
    private int periodicCallCount = 0;
    private int rightResultCount = 0;
    private int leftResultCount = 0;

    // Vision enable/disable flag for testing
    private boolean visionEnabled = true;

    /**
     * Creates a new Vision subsystem with two cameras.
     */
    public Vision() {
        // Initialize cameras with names matching PhotonVision UI
        cameraRight = new PhotonCamera(VisionConstants.CAMERA_NAME_RIGHT);
        cameraLeft = new PhotonCamera(VisionConstants.CAMERA_NAME_LEFT);

        // Check for valid field layout before creating pose estimators
        if (VisionConstants.TAG_LAYOUT == null) {
            System.err.println("WARNING: AprilTag field layout is null! Vision pose estimation will be disabled.");
            poseEstimatorRight = null;
            poseEstimatorLeft = null;
            return;
        }

        // Initialize pose estimators using MULTI_TAG_PNP_ON_COPROCESSOR strategy
        // This is the recommended strategy per PhotonVision docs for best accuracy
        poseEstimatorRight = new PhotonPoseEstimator(
                VisionConstants.TAG_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.ROBOT_TO_CAM_RIGHT);

        poseEstimatorLeft = new PhotonPoseEstimator(
                VisionConstants.TAG_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.ROBOT_TO_CAM_LEFT);

        // Set fallback strategy for when only one tag is visible
        poseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        poseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        periodicCallCount++;

        // Process all unread results from both cameras (always runs - this is the
        // critical path)
        updateCameraRight();
        updateCameraLeft();

        // Log telemetry at reduced rate to avoid loop overruns
        if (periodicCallCount % TELEMETRY_UPDATE_INTERVAL == 0) {
            logTelemetry();
        }
    }

    /**
     * Updates pose estimates from the right camera.
     */
    private void updateCameraRight() {
        latestEstimateRight = Optional.empty();

        // Skip if pose estimator wasn't initialized (no field layout)
        if (poseEstimatorRight == null) {
            return;
        }

        var results = cameraRight.getAllUnreadResults();
        rightResultCount += results.size();

        for (PhotonPipelineResult result : results) {
            latestResultRight = result;

            // Skip if no targets
            if (!result.hasTargets()) {
                continue;
            }

            // Update the pose estimator with this result
            Optional<EstimatedRobotPose> estimate = poseEstimatorRight.update(result);

            if (estimate.isPresent()) {
                // Validate the estimate before accepting
                boolean valid = isValidEstimate(estimate.get(), result.getTargets());

                if (valid) {
                    latestEstimateRight = estimate;
                    currentStdDevsRight = calculateStdDevs(estimate.get(), result.getTargets());
                }
            } else {
                // FALLBACK: If estimator fails but we have a tag in the layout, calculate
                // manually
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                int tagId = bestTarget.getFiducialId();
                var tagPoseOpt = VisionConstants.TAG_LAYOUT.getTagPose(tagId);

                if (tagPoseOpt.isPresent()) {
                    Transform3d camToTarget = bestTarget.getBestCameraToTarget();
                    Pose3d tagPose = tagPoseOpt.get();
                    Pose3d cameraPose = tagPose.transformBy(camToTarget.inverse());
                    Pose3d robotPose = cameraPose.transformBy(VisionConstants.ROBOT_TO_CAM_RIGHT.inverse());

                    latestEstimateRight = Optional.of(new EstimatedRobotPose(
                            robotPose,
                            result.getTimestampSeconds(),
                            result.getTargets(),
                            PoseStrategy.LOWEST_AMBIGUITY));
                    currentStdDevsRight = VisionConstants.SINGLE_TAG_STD_DEVS;
                }
            }
        }
    }

    /**
     * Updates pose estimates from the left camera.
     */
    private void updateCameraLeft() {
        latestEstimateLeft = Optional.empty();

        // Skip if pose estimator wasn't initialized (no field layout)
        if (poseEstimatorLeft == null) {
            return;
        }

        var results = cameraLeft.getAllUnreadResults();
        leftResultCount += results.size();

        for (PhotonPipelineResult result : results) {
            latestResultLeft = result;

            // Skip if no targets
            if (!result.hasTargets()) {
                continue;
            }

            // Update the pose estimator with this result
            Optional<EstimatedRobotPose> estimate = poseEstimatorLeft.update(result);

            if (estimate.isPresent()) {
                // Validate the estimate before accepting
                boolean valid = isValidEstimate(estimate.get(), result.getTargets());

                if (valid) {
                    latestEstimateLeft = estimate;
                    currentStdDevsLeft = calculateStdDevs(estimate.get(), result.getTargets());
                }
            } else {
                // FALLBACK: If estimator fails but we have a tag in the layout, calculate
                // manually
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                int tagId = bestTarget.getFiducialId();
                var tagPoseOpt = VisionConstants.TAG_LAYOUT.getTagPose(tagId);

                if (tagPoseOpt.isPresent()) {
                    Transform3d camToTarget = bestTarget.getBestCameraToTarget();
                    Pose3d tagPose = tagPoseOpt.get();
                    Pose3d cameraPose = tagPose.transformBy(camToTarget.inverse());
                    Pose3d robotPose = cameraPose.transformBy(VisionConstants.ROBOT_TO_CAM_LEFT.inverse());

                    latestEstimateLeft = Optional.of(new EstimatedRobotPose(
                            robotPose,
                            result.getTimestampSeconds(),
                            result.getTargets(),
                            PoseStrategy.LOWEST_AMBIGUITY));
                    currentStdDevsLeft = VisionConstants.SINGLE_TAG_STD_DEVS;
                }
            }
        }
    }

    /**
     * Validates an estimated pose to filter out bad measurements.
     */
    private boolean isValidEstimate(EstimatedRobotPose estimate, List<PhotonTrackedTarget> targets) {
        // Check if pose is within field bounds (with some margin)
        Pose3d pose = estimate.estimatedPose;
        if (pose.getX() < -1 || pose.getX() > 17 ||
                pose.getY() < -1 || pose.getY() > 9 ||
                pose.getZ() < -0.5 || pose.getZ() > 1.5) {
            return false;
        }

        // For single-tag estimates, check ambiguity
        if (targets.size() == 1) {
            PhotonTrackedTarget target = targets.get(0);
            if (target.getPoseAmbiguity() > VisionConstants.MAX_AMBIGUITY) {
                return false;
            }

            // Check distance - single tags are unreliable at long range
            double distance = target.getBestCameraToTarget().getTranslation().getNorm();
            if (distance > VisionConstants.MAX_VISION_DISTANCE) {
                return false;
            }
        }

        return true;
    }

    /**
     * Calculates dynamic standard deviations based on target quality.
     */
    private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose estimate, List<PhotonTrackedTarget> targets) {
        int numTargets = targets.size();

        // Multi-tag estimates are much more reliable
        if (numTargets >= VisionConstants.MIN_MULTI_TAG_TARGETS) {
            return VisionConstants.MULTI_TAG_STD_DEVS;
        }

        // Single tag - scale std devs by distance
        if (numTargets == 1) {
            PhotonTrackedTarget target = targets.get(0);
            double distance = target.getBestCameraToTarget().getTranslation().getNorm();
            double ambiguity = target.getPoseAmbiguity();

            double scaleFactor = distance * (1 + ambiguity * 5);

            return VecBuilder.fill(
                    VisionConstants.SINGLE_TAG_STD_DEVS.get(0, 0) * scaleFactor,
                    VisionConstants.SINGLE_TAG_STD_DEVS.get(1, 0) * scaleFactor,
                    VisionConstants.SINGLE_TAG_STD_DEVS.get(2, 0) * scaleFactor);
        }

        return VisionConstants.SINGLE_TAG_STD_DEVS;
    }

    // ==================
    // LEFT CAMERA TARGETING METHODS (for blind-spot-free alignment)
    // ==================

    /**
     * Gets the yaw to target using LEFT camera as the alignment reference.
     * When this yaw is 0, the left camera is directly facing the AprilTag.
     * 
     * This solves the blind spot problem - the robot rotates until the LEFT camera
     * faces the target, keeping the tag visible throughout the rotation.
     * 
     * @return yaw in degrees from left camera's perspective (positive = target is
     *         to camera's left)
     */
    public double getLeftCameraTargetYaw() {
        if (latestResultLeft != null && latestResultLeft.hasTargets()) {
            return latestResultLeft.getBestTarget().getYaw();
        }
        return 0.0;
    }

    /**
     * Gets the best target from the LEFT camera specifically.
     * Use this for alignment commands that want to keep the tag in left camera's
     * view.
     */
    public Optional<PhotonTrackedTarget> getLeftCameraBestTarget() {
        if (latestResultLeft != null && latestResultLeft.hasTargets()) {
            return Optional.of(latestResultLeft.getBestTarget());
        }
        return Optional.empty();
    }

    /**
     * Checks if the LEFT camera currently sees any targets.
     */
    public boolean leftCameraHasTargets() {
        return latestResultLeft != null && latestResultLeft.hasTargets();
    }

    // ==================
    // GENERAL TARGET METHODS
    // ==================

    /**
     * Gets the latest estimated pose from the right camera.
     */
    public Optional<EstimatedRobotPose> getEstimatedPoseRight() {
        if (!visionEnabled) {
            return Optional.empty();
        }
        return latestEstimateRight;
    }

    /**
     * Gets the latest estimated pose from the left camera.
     */
    public Optional<EstimatedRobotPose> getEstimatedPoseLeft() {
        if (!visionEnabled) {
            return Optional.empty();
        }
        return latestEstimateLeft;
    }

    /**
     * Gets the standard deviations for the right camera's latest estimate.
     */
    public Matrix<N3, N1> getStdDevsRight() {
        return currentStdDevsRight;
    }

    /**
     * Gets the standard deviations for the left camera's latest estimate.
     */
    public Matrix<N3, N1> getStdDevsLeft() {
        return currentStdDevsLeft;
    }

    /**
     * Gets the latest pipeline result from the right camera.
     */
    public PhotonPipelineResult getLatestResultRight() {
        return latestResultRight;
    }

    /**
     * Gets the latest pipeline result from the left camera.
     */
    public PhotonPipelineResult getLatestResultLeft() {
        return latestResultLeft;
    }

    /**
     * Enum to identify which camera detected a target.
     */
    public enum CameraSource {
        RIGHT,
        LEFT,
        NONE
    }

    /**
     * Result containing a target and which camera saw it.
     */
    public record TargetWithSource(PhotonTrackedTarget target, CameraSource source) {
    }

    /**
     * Gets the best target from either camera (prefers lowest ambiguity).
     */
    public Optional<PhotonTrackedTarget> getBestTarget() {
        return getBestTargetWithSource().map(TargetWithSource::target);
    }

    /**
     * Gets the best target along with which camera detected it.
     */
    public Optional<TargetWithSource> getBestTargetWithSource() {
        PhotonTrackedTarget bestTarget = null;
        double bestAmbiguity = Double.MAX_VALUE;
        CameraSource source = CameraSource.NONE;

        // Check right camera
        if (latestResultRight != null && latestResultRight.hasTargets()) {
            for (PhotonTrackedTarget target : latestResultRight.getTargets()) {
                if (target.getPoseAmbiguity() < bestAmbiguity) {
                    bestAmbiguity = target.getPoseAmbiguity();
                    bestTarget = target;
                    source = CameraSource.RIGHT;
                }
            }
        }

        // Check left camera
        if (latestResultLeft != null && latestResultLeft.hasTargets()) {
            for (PhotonTrackedTarget target : latestResultLeft.getTargets()) {
                if (target.getPoseAmbiguity() < bestAmbiguity) {
                    bestAmbiguity = target.getPoseAmbiguity();
                    bestTarget = target;
                    source = CameraSource.LEFT;
                }
            }
        }

        if (bestTarget != null) {
            return Optional.of(new TargetWithSource(bestTarget, source));
        }
        return Optional.empty();
    }

    /**
     * Gets a specific target by its AprilTag ID from either camera.
     */
    public Optional<PhotonTrackedTarget> getTargetById(int tagId) {
        // Check right camera
        if (latestResultRight != null && latestResultRight.hasTargets()) {
            for (PhotonTrackedTarget target : latestResultRight.getTargets()) {
                if (target.getFiducialId() == tagId) {
                    return Optional.of(target);
                }
            }
        }

        // Check left camera
        if (latestResultLeft != null && latestResultLeft.hasTargets()) {
            for (PhotonTrackedTarget target : latestResultLeft.getTargets()) {
                if (target.getFiducialId() == tagId) {
                    return Optional.of(target);
                }
            }
        }

        return Optional.empty();
    }

    /**
     * Checks if any target is currently visible on either camera.
     */
    public boolean hasTargets() {
        boolean rightHas = latestResultRight != null && latestResultRight.hasTargets();
        boolean leftHas = latestResultLeft != null && latestResultLeft.hasTargets();
        return rightHas || leftHas;
    }

    /**
     * Sets the reference pose for CLOSEST_TO_REFERENCE_POSE strategy.
     */
    public void setReferencePose(Pose2d pose) {
        if (poseEstimatorRight != null) {
            poseEstimatorRight.setReferencePose(pose);
        }
        if (poseEstimatorLeft != null) {
            poseEstimatorLeft.setReferencePose(pose);
        }
    }

    /**
     * Enables or disables driver mode on both cameras.
     */
    public void setDriverMode(boolean enabled) {
        cameraRight.setDriverMode(enabled);
        cameraLeft.setDriverMode(enabled);
    }

    /**
     * Logs essential vision telemetry to SmartDashboard (throttled).
     */
    private void logTelemetry() {
        SmartDashboard.putBoolean("Vision/Enabled", visionEnabled);

        // Right camera status
        boolean rightConnected = cameraRight.isConnected();
        boolean rightHasTargets = latestResultRight != null && latestResultRight.hasTargets();
        SmartDashboard.putBoolean("Vision/Right/Connected", rightConnected);
        SmartDashboard.putBoolean("Vision/Right/HasTargets", rightHasTargets);

        if (rightHasTargets) {
            SmartDashboard.putNumber("Vision/Right/NumTargets", latestResultRight.getTargets().size());
            SmartDashboard.putNumber("Vision/Right/BestYaw", latestResultRight.getBestTarget().getYaw());
        }

        // Left camera status
        boolean leftConnected = cameraLeft.isConnected();
        boolean leftHasTargets = latestResultLeft != null && latestResultLeft.hasTargets();
        SmartDashboard.putBoolean("Vision/Left/Connected", leftConnected);
        SmartDashboard.putBoolean("Vision/Left/HasTargets", leftHasTargets);

        if (leftHasTargets) {
            SmartDashboard.putNumber("Vision/Left/NumTargets", latestResultLeft.getTargets().size());
            SmartDashboard.putNumber("Vision/Left/BestYaw", latestResultLeft.getBestTarget().getYaw());
        }

        // Overall best target
        Optional<PhotonTrackedTarget> best = getBestTarget();
        SmartDashboard.putBoolean("Vision/HasTarget", best.isPresent());
        if (best.isPresent()) {
            SmartDashboard.putNumber("Vision/BestTarget/ID", best.get().getFiducialId());
            SmartDashboard.putNumber("Vision/BestTarget/Yaw", best.get().getYaw());
            SmartDashboard.putNumber("Vision/BestTarget/Pitch", best.get().getPitch());
        }

        // Pose estimates
        latestEstimateRight.ifPresent(est -> {
            Pose2d pose = est.estimatedPose.toPose2d();
            SmartDashboard.putNumber("Vision/Right/EstX", pose.getX());
            SmartDashboard.putNumber("Vision/Right/EstY", pose.getY());
            SmartDashboard.putNumber("Vision/Right/EstRot", pose.getRotation().getDegrees());
        });

        latestEstimateLeft.ifPresent(est -> {
            Pose2d pose = est.estimatedPose.toPose2d();
            SmartDashboard.putNumber("Vision/Left/EstX", pose.getX());
            SmartDashboard.putNumber("Vision/Left/EstY", pose.getY());
            SmartDashboard.putNumber("Vision/Left/EstRot", pose.getRotation().getDegrees());
        });
    }

    // ==================
    // VISION CONTROL METHODS
    // ==================

    public void enableVision() {
        visionEnabled = true;
    }

    public void disableVision() {
        visionEnabled = false;
    }

    public void toggleVision() {
        visionEnabled = !visionEnabled;
    }

    public boolean isVisionEnabled() {
        return visionEnabled;
    }
}
