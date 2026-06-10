package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

/** Real turret camera. */
public class VisionIOReal implements VisionIO {
  private final PhotonCamera camera = new PhotonCamera(VisionConstants.TURRET_CAM_NAME);

  @Override
  public VisionIOInputs updateInputs() {
    return new VisionIOInputs(camera.isConnected(), camera.getAllUnreadResults());
  }
}
