package frc.robot.runtime.perception;

/**
 * A perception provider that sees nothing — used when the vision feature flag is off. The turret
 * never gets a fresh frame (so it holds), aiming never reads "aimed", and the feed gate never
 * opens: a safe, inert default with no camera present.
 */
public final class NoPerception implements PerceptionProvider {
  @Override
  public boolean isResultFresh() {
    return false;
  }

  @Override
  public double getResultTimestamp() {
    return 0.0;
  }

  @Override
  public boolean isSeeingHubTag() {
    return false;
  }

  @Override
  public double getYawToHubDeg() {
    return 0.0;
  }

  @Override
  public double getDistanceToHub() {
    return 0.0;
  }

  @Override
  public int getTrackedTagId() {
    return -1;
  }

  @Override
  public boolean isSeeingPassingTag() {
    return false;
  }

  @Override
  public double getYawToPassingTagDeg() {
    return 0.0;
  }

  @Override
  public double getDistanceToPassingTag() {
    return 0.0;
  }

  @Override
  public int getTrackedPassingTagId() {
    return -1;
  }
}
