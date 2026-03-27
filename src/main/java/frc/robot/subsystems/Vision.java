package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;

/**
 * Vision subsystem: turret camera only.
 * Provides tx (yaw error) and distance-to-target for turret tracking.
 */
@Logged
public class Vision extends SubsystemBase {

    private final PhotonCamera cameraTurret;
    private final PhotonPoseEstimator poseEstimatorTurret;

    private PhotonPipelineResult latestResultTurret;
    private Supplier<Rotation2d> turretAngleSupplier = () -> new Rotation2d();

    private boolean turretResultFreshThisCycle = false;
    private double turretResultTimestamp = 0.0;

    @Logged(importance = Logged.Importance.CRITICAL) private boolean turretCamConnected = false;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean turretHasTargets = false;
    @Logged(importance = Logged.Importance.CRITICAL) private double correctedBumperDist = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double targetPitchDeg = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double visionLatencyMs = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double timeSinceLastFrameMs = 0.0;

    public Vision() {
        cameraTurret = VisionConstants.ENABLE_TURRET_CAM
                ? new PhotonCamera(VisionConstants.TURRET_CAM_NAME) : null;

        if (VisionConstants.TAG_LAYOUT == null || cameraTurret == null) {
            poseEstimatorTurret = null;
            return;
        }

        poseEstimatorTurret = new PhotonPoseEstimator(
                VisionConstants.TAG_LAYOUT,
                calculateTurretCameraTransform(new Rotation2d()));
    }

    @Override
    public void periodic() {
        updateTurretCameraTransform();

        turretResultFreshThisCycle = false;
        if (cameraTurret != null) {
            for (PhotonPipelineResult result : cameraTurret.getAllUnreadResults()) {
                latestResultTurret = result;
                turretResultFreshThisCycle = true;
                turretResultTimestamp = result.getTimestampSeconds();
            }
        }

        turretCamConnected = cameraTurret != null && cameraTurret.isConnected();
        turretHasTargets = latestResultTurret != null && latestResultTurret.hasTargets();

        updateTurretDiagnostics();
    }

    // ── Turret camera transform (dynamic, updates each cycle) ──

    private Transform3d calculateTurretCameraTransform(Rotation2d turretAngle) {
        Transform3d turretRotation = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, 0, turretAngle.getRadians()));
        return VisionConstants.ROBOT_TO_TURRET_PIVOT
                .plus(turretRotation)
                .plus(VisionConstants.TURRET_PIVOT_TO_CAM);
    }

    private void updateTurretCameraTransform() {
        if (poseEstimatorTurret != null) {
            poseEstimatorTurret.setRobotToCameraTransform(
                    calculateTurretCameraTransform(turretAngleSupplier.get()));
        }
    }

    public void setTurretAngleSupplier(Supplier<Rotation2d> supplier) {
        this.turretAngleSupplier = supplier;
    }

    // ── Turret camera accessors (used by TurretAutoAimCommand) ──

    public double getTurretCameraDistanceToTarget() {
        if (latestResultTurret == null || !latestResultTurret.hasTargets()) return 0.0;
        Translation3d camToTag = latestResultTurret.getBestTarget()
                .getBestCameraToTarget().getTranslation();
        double hDist = Math.hypot(camToTag.getX(), camToTag.getY());
        return hDist + VisionConstants.CAMERA_TO_BUMPER_OFFSET_METERS;
    }

    public Optional<PhotonTrackedTarget> getTurretCameraBestTarget() {
        if (latestResultTurret != null && latestResultTurret.hasTargets()) {
            return Optional.of(latestResultTurret.getBestTarget());
        }
        return Optional.empty();
    }

    public boolean turretCameraHasTargets() {
        return latestResultTurret != null && latestResultTurret.hasTargets();
    }

    public boolean isTurretResultFresh() {
        return turretResultFreshThisCycle;
    }

    public double getTurretResultTimestamp() {
        return turretResultTimestamp;
    }

    // ── Diagnostics ──

    private void updateTurretDiagnostics() {
        if (latestResultTurret != null && latestResultTurret.hasTargets()) {
            correctedBumperDist = getTurretCameraDistanceToTarget();
            targetPitchDeg = latestResultTurret.getBestTarget().getPitch();
        }
        if (turretResultTimestamp > 0) {
            timeSinceLastFrameMs = (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - turretResultTimestamp) * 1000.0;
        }
        if (turretResultFreshThisCycle && latestResultTurret != null) {
            visionLatencyMs = latestResultTurret.metadata.getLatencyMillis();
        }
    }
}
