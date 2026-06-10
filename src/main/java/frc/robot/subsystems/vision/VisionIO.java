package frc.robot.subsystems.vision;

import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Hardware boundary for the turret camera. PhotonPipelineResult is pure DATA (PhotonVision's
 * serialized frame record), not a hardware handle — passing it through the seam keeps the
 * targeting pipeline and the pose estimator byte-faithful while real and sim stay swappable.
 */
public interface VisionIO {

  record VisionIOInputs(boolean connected, List<PhotonPipelineResult> unreadResults) {
    public static final VisionIOInputs kEmpty = new VisionIOInputs(false, List.of());
  }

  /** Drain and return every unread frame since last loop. */
  VisionIOInputs updateInputs();
}
