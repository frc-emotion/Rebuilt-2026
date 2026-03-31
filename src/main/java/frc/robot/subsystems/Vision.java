package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.epilogue.Logged;
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

    @Logged(importance = Logged.Importance.CRITICAL) private boolean cameraConnected = false;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean seeingHubTag = false;
    @Logged(importance = Logged.Importance.CRITICAL) private double distanceToHub = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double yawToHubDeg = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private int trackedTagId = -1;
    @Logged(importance = Logged.Importance.CRITICAL) private double latencyMs = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double ambiguity = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double rawDeg = 0.0;

    public Vision() {
        camera = VisionConstants.ENABLE_TURRET_CAM
                ? new PhotonCamera(VisionConstants.TURRET_CAM_NAME)
                : null;
    }

    @Override
    public void periodic() {
        // 1. Drain all unread frames, keep the newest one
        freshThisCycle = false;
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
            seeingHubTag = false;
            distanceToHub = lastGoodDistance;
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
                if (!VisionConstants.BENCH_TEST_ANY_TAG && !VisionConstants.isOurHubTag(id)) continue;
                double amb = target.getPoseAmbiguity();
                if (!VisionConstants.BENCH_TEST_ANY_TAG && amb > VisionConstants.MAX_POSE_AMBIGUITY) continue;
                if (amb < bestAmbiguity) {
                    bestTarget = target;
                    bestAmbiguity = amb;
                }
            }
        }

        if (bestTarget == null) {
            seeingHubTag = false;
            distanceToHub = lastGoodDistance;
            return;
        }

        // 4. Compute distance + yaw
        int tagId = bestTarget.getFiducialId();

        if (VisionConstants.BENCH_TEST_ANY_TAG) {
            // Bench mode: use raw PhotonVision yaw + raw camera→tag distance
            // This matches pre-refactor behavior exactly
            yawToHubDeg = bestTarget.getYaw();
            Translation3d camToTag = bestTarget.getBestCameraToTarget().getTranslation();
            distanceToHub = Math.hypot(camToTag.getX(), camToTag.getY());
        } else {
            rawDeg = bestTarget.getYaw();
            // Competition mode: camera→tag + tag→hub vector addition
            Transform3d cameraToTag = bestTarget.getBestCameraToTarget();
            Transform3d tagToHub = VisionConstants.TAG_TO_HUB_CENTER.get(tagId);

            Translation3d toHub = (tagToHub != null)
                    ? cameraToTag.plus(tagToHub).getTranslation()
                    : cameraToTag.getTranslation();

            distanceToHub = Math.hypot(toHub.getX(), toHub.getY());
            yawToHubDeg = -Math.toDegrees(Math.atan2(toHub.getY(), toHub.getX()));
        }

        // 5. Update state
        trackedTagId = tagId;
        ambiguity = bestAmbiguity;
        latencyMs = latestResult.metadata.getLatencyMillis();
        seeingHubTag = true;
        lastGoodDistance = distanceToHub;
    }

    // ── Public API ──

    /** Horizontal distance from camera to hub center in meters. Holds last good value when no tag visible. */
    public double getDistanceToHub() {
        return distanceToHub;
    }

    /** Degrees the hub center is off the camera's forward axis. 0 when no tag visible. */
    public double getYawToHubDeg() {
        return yawToHubDeg;
    }

    /** True when a valid hub tag (or any tag in bench mode) passed ambiguity check this cycle. */
    public boolean isSeeingHubTag() {
        return seeingHubTag;
    }

    /** True when the camera delivered a new frame this cycle. */
    public boolean isTurretResultFresh() {
        return freshThisCycle;
    }

    /** Timestamp of the latest camera frame (for dedup in TurretAutoAimCommand). */
    public double getTurretResultTimestamp() {
        return resultTimestamp;
    }

    /** The fiducial ID currently being tracked (-1 if none). */
    public int getTrackedTagId() {
        return trackedTagId;
    }
}
