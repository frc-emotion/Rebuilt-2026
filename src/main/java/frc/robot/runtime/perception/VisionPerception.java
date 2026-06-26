package frc.robot.runtime.perception;

import frc.robot.subsystems.vision.Vision;

/**
 * The local {@link PerceptionProvider}: a thin adapter over the existing turret-camera {@code
 * Vision} subsystem (kept as-is — targeting verbatim, pose estimation still gated off). Every
 * method delegates 1:1 to the legacy Vision public surface, so the ported {@code TurretAiming}
 * reads exactly what it read before. This is the seam where a coprocessor estimate later
 * substitutes for the local camera without touching aiming or the skills.
 */
public final class VisionPerception implements PerceptionProvider {
  private final Vision vision;

  public VisionPerception(Vision vision) {
    this.vision = vision;
  }

  @Override
  public boolean isResultFresh() {
    return vision.isResultFresh();
  }

  @Override
  public double getResultTimestamp() {
    return vision.getResultTimestamp();
  }

  @Override
  public boolean isSeeingHubTag() {
    return vision.isSeeingHubTag();
  }

  @Override
  public double getYawToHubDeg() {
    return vision.getYawToHubDeg();
  }

  @Override
  public double getDistanceToHub() {
    return vision.getDistanceToHub();
  }

  @Override
  public int getTrackedTagId() {
    return vision.getTrackedTagId();
  }

  @Override
  public boolean isSeeingPassingTag() {
    return vision.isSeeingPassingTag();
  }

  @Override
  public double getYawToPassingTagDeg() {
    return vision.getYawToPassingTagDeg();
  }

  @Override
  public double getDistanceToPassingTag() {
    return vision.getDistanceToPassingTag();
  }

  @Override
  public int getTrackedPassingTagId() {
    return vision.getTrackedPassingTagId();
  }
}
