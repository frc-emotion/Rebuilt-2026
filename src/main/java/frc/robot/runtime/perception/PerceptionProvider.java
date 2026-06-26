package frc.robot.runtime.perception;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

/**
 * The perception seam. Today the only implementation wraps the local turret-camera {@code Vision}
 * subsystem (targeting + the pose-estimation path, still gated OFF); tomorrow a coprocessor
 * estimate can implement the same interface and drop in without touching the setpoint functions or
 * reflexes.
 *
 * <p>The hub/passing surface is exactly the legacy {@code Vision} public API the aiming brain reads
 * (verbatim names + semantics), so the ported {@code TurretAiming} consumes a provider instead of a
 * subsystem. Sign conventions, the exact-timestamp dedupe, the {@code TAG_TO_HUB_CENTER} fudge, and
 * the hub-before-passing classification all live in the implementation, unchanged.
 */
public interface PerceptionProvider {

  /** True when a new camera frame was drained this loop (legacy {@code isResultFresh}). */
  boolean isResultFresh();

  /** Seconds timestamp of the latest frame — the key for the exact-timestamp dedupe. */
  double getResultTimestamp();

  // ── Hub targeting ──

  boolean isSeeingHubTag();

  double getYawToHubDeg();

  double getDistanceToHub();

  int getTrackedTagId();

  // ── Passing targeting ──

  boolean isSeeingPassingTag();

  double getYawToPassingTagDeg();

  double getDistanceToPassingTag();

  int getTrackedPassingTagId();

  // ── State-estimate seam (inert locally: pose estimation is hard-gated off until the
  // camera-on-turret transforms are measured). A coprocessor estimate plugs in here later. ──

  /**
   * The fused field pose, if perception produces one. Empty locally (pose estimation gated off).
   */
  default Optional<Pose2d> getEstimatedPose() {
    return Optional.empty();
  }
}
