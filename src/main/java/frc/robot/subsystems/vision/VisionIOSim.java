package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Photonlib VisionSystemSim drives a simulated turret camera from the simulated drive pose and
 * the simulated turret angle (the camera transform is adjusted every frame — the documented
 * photonlib pattern for cameras on moving mechanisms).
 */
public class VisionIOSim implements VisionIO {
  private final PhotonCamera camera = new PhotonCamera(VisionConstants.TURRET_CAM_NAME);
  private final PhotonCameraSim cameraSim;
  private final VisionSystemSim visionSim = new VisionSystemSim("turret");

  private final Supplier<Pose2d> robotPoseSupplier;
  private final DoubleSupplier turretAngleRotSupplier;

  public VisionIOSim(
      AprilTagFieldLayout fieldLayout,
      Supplier<Pose2d> robotPoseSupplier,
      DoubleSupplier turretAngleRotSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.turretAngleRotSupplier = turretAngleRotSupplier;

    visionSim.addAprilTags(fieldLayout);
    SimCameraProperties properties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, properties);
    visionSim.addCamera(cameraSim, robotToCamera(0.0));
  }

  @Override
  public VisionIOInputs updateInputs() {
    visionSim.adjustCamera(cameraSim, robotToCamera(turretAngleRotSupplier.getAsDouble()));
    visionSim.update(robotPoseSupplier.get());
    return new VisionIOInputs(camera.isConnected(), camera.getAllUnreadResults());
  }

  private static Transform3d robotToCamera(double turretRot) {
    return VisionConstants.ROBOT_TO_TURRET
        .plus(new Transform3d(
            new edu.wpi.first.math.geometry.Translation3d(),
            new Rotation3d(0, 0, edu.wpi.first.math.util.Units.rotationsToRadians(turretRot))))
        .plus(VisionConstants.TURRET_TO_CAMERA);
  }
}
