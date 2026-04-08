package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Vision subsystem: single turret-mounted camera.
 *
 * Every cycle this subsystem computes two numbers:
 *   1. distanceToHub — horizontal meters from camera to hub center
 *   2. yawToHubDeg   — degrees the hub center is off the camera's forward axis
 *
 * The math is one vector addition:
 *   camera→hub = camera→tag (from PhotonVision) + tag→hub (fixed constant)
 *
 * BENCH_TEST_ANY_TAG mode skips the tag→hub offset so you can aim at any
 * wall tag without needing a real hub.
 */
@Logged
public class Vision extends SubsystemBase {

    private final PhotonCamera camera;

    private PhotonPipelineResult latestResult;
    private boolean freshThisCycle = false;
    private double resultTimestamp = 0.0;

    // Stale-data hold: distance keeps its last good value when no tag is visible
    private double lastGoodDistance = 0.0;
    private double lastGoodPassingDistance = 0.0;

    @Logged(importance = Logged.Importance.CRITICAL) private boolean cameraConnected = false;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean seeingHubTag = false;
    @Logged(importance = Logged.Importance.CRITICAL) private double distanceToHub = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double yawToHubDeg = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private int trackedTagId = -1;
    @Logged(importance = Logged.Importance.CRITICAL) private double latencyMs = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double ambiguity = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double rawDeg = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean seeingPassingTag = false;
    @Logged(importance = Logged.Importance.CRITICAL) private double distanceToPassingTag = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double yawToPassingTagDeg = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private int trackedPassingTagId = -1;

    public Vision() {
        camera = VisionConstants.ENABLE_TURRET_CAM
                ? new PhotonCamera(VisionConstants.TURRET_CAM_NAME)
                : null;
    }

    @Override
    public void periodic() {
        // 1. Drain all unread frames, keep the newest one
        freshThisCycle = false;
        seeingHubTag = false;    // FIX 3: reset both flags every cycle up here
        seeingPassingTag = false;
        if (camera != null) {
            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                latestResult = result;
                freshThisCycle = true;
                resultTimestamp = result.getTimestampSeconds();
            }
        }

        cameraConnected = camera != null && camera.isConnected();

        // 2. Nothing new or no targets? Hold stale distance, clear tracking flag
        if (!freshThisCycle || latestResult == null || !latestResult.hasTargets()) {
            distanceToHub = lastGoodDistance;
            distanceToPassingTag = lastGoodPassingDistance;
            return;
        }

        // 3. Find target — prefer the tag we're already tracking (sticky)
        PhotonTrackedTarget bestTarget = null;
        double bestAmbiguity = 1.0;

        // First pass: look for the tag we're already tracking
        if (trackedTagId != -1) {
            for (PhotonTrackedTarget target : latestResult.getTargets()) {
                if (target.getFiducialId() != trackedTagId) continue;
                double amb = target.getPoseAmbiguity();
                if (!VisionConstants.BENCH_TEST_ANY_TAG && amb > VisionConstants.MAX_POSE_AMBIGUITY) break;
                bestTarget = target;
                bestAmbiguity = amb;
                break;
            }
        }

        // Second pass: current tag not found — pick the best new one
        if (bestTarget == null) {
            for (PhotonTrackedTarget target : latestResult.getTargets()) {
                int id = target.getFiducialId();
                if (!VisionConstants.BENCH_TEST_ANY_TAG && !VisionConstants.isOurHubTag(id) && !VisionConstants.isOurPassingTag(id)) continue;
                double amb = target.getPoseAmbiguity();
                if (!VisionConstants.BENCH_TEST_ANY_TAG && amb > VisionConstants.MAX_POSE_AMBIGUITY) continue;
                if (amb < bestAmbiguity) {
                    bestTarget = target;
                    bestAmbiguity = amb;
                }
            }
        }

        // FIX 1: return early if no valid target found
        if (bestTarget == null) {
            distanceToHub = lastGoodDistance;
            distanceToPassingTag = lastGoodPassingDistance;
            return;
        }

        // FIX 2: declare tagId once, use it for classification
        int tagId = bestTarget.getFiducialId();

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
        } else {
            if (seeingHubTag) {
                rawDeg = bestTarget.getYaw();
                Transform3d cameraToTag = bestTarget.getBestCameraToTarget();
                Transform3d tagToHub = VisionConstants.TAG_TO_HUB_CENTER.get(tagId);

                Translation3d toHub = (tagToHub != null)
                        ? cameraToTag.plus(tagToHub).getTranslation()
                        : cameraToTag.getTranslation();

                distanceToHub = Math.hypot(toHub.getX(), toHub.getY());
                yawToHubDeg = -Math.toDegrees(Math.atan2(toHub.getY(), toHub.getX()));
            } else if (seeingPassingTag) {
                rawDeg = bestTarget.getYaw();
                Transform3d cameraToTag = bestTarget.getBestCameraToTarget();

                // FIX 5: write to distanceToPassingTag, not distanceToHub
                distanceToPassingTag = Math.hypot(cameraToTag.getTranslation().getX(), cameraToTag.getTranslation().getY());

                Rotation3d tagToCamRot = cameraToTag.getRotation().unaryMinus();
                Translation3d tagNormalInCam = new Translation3d(0, 0, 1).rotateBy(tagToCamRot);
                yawToPassingTagDeg = -Math.toDegrees(
                        Math.atan2(-tagNormalInCam.getY(), -tagNormalInCam.getX()));
            }
        }

        // 5. Update state
        trackedTagId = tagId;
        ambiguity = bestAmbiguity;
        latencyMs = latestResult.metadata.getLatencyMillis();

        // FIX 3 & 4: only update the relevant stale holder, write trackedPassingTagId
        if (seeingHubTag) {
            lastGoodDistance = distanceToHub;
        }
        if (seeingPassingTag) {
            trackedPassingTagId = tagId;
            lastGoodPassingDistance = distanceToPassingTag;
        }
    }

    // ── Public API ──

    public double getDistanceToHub() {
        return distanceToHub;
    }

    public double getYawToHubDeg() {
        return yawToHubDeg;
    }

    public boolean isSeeingHubTag() {
        return seeingHubTag;
    }

    public boolean isTurretResultFresh() {
        return freshThisCycle;
    }

    public double getTurretResultTimestamp() {
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