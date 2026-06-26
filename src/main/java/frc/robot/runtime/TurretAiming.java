package frc.robot.runtime;

import frc.robot.runtime.TurretWrap.WrapResult;
import frc.robot.runtime.perception.PerceptionProvider;
import frc.robot.runtime.reflex.ReflexConstants;

/**
 * The aiming brain, ported VERBATIM from the legacy superstructure TurretAiming (itself ported from
 * TurretAutoAimCommand): pure visual servoing (target = current position + camera tx) carried
 * between frames by gyro-delta feedforward. The ONLY change from the legacy class is the seam — it
 * reads a {@link PerceptionProvider} and takes the current turret rotation + soft limits as
 * arguments, and RETURNS the wrapped/clamped commanded target instead of calling a Turret
 * subsystem. The caller commands the turret mechanism with the returned value (turret has no
 * further clamp, so the returned value IS the commanded value — the wrap feedback, W2, stays
 * exact).
 *
 * <p>This is a turret-tracking reflex: stateful (gyro reference, dedupe timestamp, accumulated
 * target) and real-time. Every value (3° aim deadband, the ADDITIVE gyro sign W3, the
 * exact-timestamp dedupe W5, the W10 first-fresh-frame guard) is preserved bit-for-bit.
 */
public class TurretAiming {

  private double targetPositionRot = 0.0;

  private double visionTxDeg = 0.0;

  private boolean visionActive = false;

  private double distanceToHubMeters = 0.0;

  private int trackedTagId = -1;

  private boolean passingDirectionLocked = false;

  private double gyroFeedforwardRot = 0.0;

  private double omegaRadPerSec = 0.0;

  private double lastGyroYawDeg = 0.0;
  private double lastVisionTimestamp = 0.0;
  private boolean freshVisionThisCycle = false;
  // W10 fix: before the first fresh hub frame, visionTxDeg is 0 and would read as "aimed".
  private boolean seenFreshHubFrameSinceReset = false;

  /** Re-sync on enable and on manual-mode exit: hold in place, re-seed the gyro reference. */
  public void reset(double currentTurretPositionRot, double currentYawDeg) {
    targetPositionRot = currentTurretPositionRot;
    lastGyroYawDeg = currentYawDeg;
    lastVisionTimestamp = 0.0;
    visionTxDeg = 0.0;
    freshVisionThisCycle = false;
    passingDirectionLocked = false;
    seenFreshHubFrameSinceReset = false;
  }

  /**
   * Hub tracking, one loop: gyro FF + fresh-frame visual servo + wrap-fed-back command (W2).
   * Returns the wrapped/clamped target the caller must command to the turret mechanism.
   */
  public double trackHub(
      double currentTurretPositionRot,
      PerceptionProvider vision,
      double yawDeg,
      double omegaRadPerSec,
      double reverseLimitRot,
      double forwardLimitRot) {
    this.omegaRadPerSec = omegaRadPerSec; // sampled + logged; compensation deliberately off (W4)
    applyGyroFf(yawDeg);
    readHubVision(vision);
    if (freshVisionThisCycle) {
      targetPositionRot = currentTurretPositionRot + visionTxDeg / 360.0;
      seenFreshHubFrameSinceReset = true;
    }
    return commandWrapped(reverseLimitRot, forwardLimitRot);
  }

  /** Passing tracking: same loop shape, passing-tag vision, direction latch (legacy semantics). */
  public double trackPassing(
      double currentTurretPositionRot,
      PerceptionProvider vision,
      double yawDeg,
      double reverseLimitRot,
      double forwardLimitRot) {
    applyGyroFf(yawDeg);
    readPassingVision(vision);
    if (freshVisionThisCycle) {
      targetPositionRot = currentTurretPositionRot + visionTxDeg / 360.0;
      passingDirectionLocked = true;
    }
    return commandWrapped(reverseLimitRot, forwardLimitRot);
  }

  // Wrap+clamp exactly as the legacy Turret subsystem did, store the actually-commanded value back
  // (W2: integrating callers must store the returned value or their accumulator winds up), and
  // return it for the caller to command. The turret mechanism has no further output clamp, so the
  // value returned here is the value the motor is commanded with.
  private double commandWrapped(double reverseLimitRot, double forwardLimitRot) {
    WrapResult result = TurretWrap.apply(targetPositionRot, reverseLimitRot, forwardLimitRot);
    targetPositionRot = result.commandedRot();
    return targetPositionRot;
  }

  /** Manual rebase (W7): the closed-loop target follows the actual position during manual jog. */
  public void rebaseToCurrent(double currentTurretPositionRot) {
    targetPositionRot = currentTurretPositionRot;
  }

  /** Aim gate: last vision tx < 3° AND at least one fresh hub frame since reset (W10 fix). */
  public boolean isAimed() {
    return seenFreshHubFrameSinceReset && Math.abs(visionTxDeg) < ReflexConstants.kAimedDeadbandDeg;
  }

  /** Last fresh-frame hub distance — held between frames (legacy command semantics). */
  public double getDistanceToHub() {
    return distanceToHubMeters;
  }

  public boolean isPassingLocked() {
    return passingDirectionLocked;
  }

  public double getTargetPositionRot() {
    return targetPositionRot;
  }

  // Integrate measured yaw DELTAS into the setpoint so the turret stays field-pointed between
  // vision frames. The ADDITION (not subtraction) encodes the turret/gyro sign convention and is
  // load-bearing (W3). A rate-based version with a 1.75 lead fudge was tried and abandoned.
  private void applyGyroFf(double currentYawDeg) {
    gyroFeedforwardRot = 0.0;
    if (!ReflexConstants.kGyroFfEnabled) {
      return;
    }
    double deltaDeg = currentYawDeg - lastGyroYawDeg;
    lastGyroYawDeg = currentYawDeg;
    gyroFeedforwardRot = deltaDeg / 360.0;
    targetPositionRot += gyroFeedforwardRot;
  }

  // Three gates, legacy-exact (W5): pipeline freshness, exact-timestamp dedupe (a ~25 Hz camera
  // frame must not be applied twice by the 50 Hz loop), then tag-identity. The timestamp is
  // consumed BEFORE the tag check — a fresh frame without the wanted tag burns the timestamp,
  // and hub/passing share the dedupe state. Legacy quirk, preserved.
  private void readHubVision(PerceptionProvider vision) {
    freshVisionThisCycle = false;
    visionActive = false;
    if (vision == null || !vision.isResultFresh()) {
      return;
    }
    double ts = vision.getResultTimestamp();
    if (ts == lastVisionTimestamp) {
      return;
    }
    lastVisionTimestamp = ts;

    if (!vision.isSeeingHubTag()) {
      return;
    }
    visionTxDeg = vision.getYawToHubDeg();
    distanceToHubMeters = vision.getDistanceToHub();
    trackedTagId = vision.getTrackedTagId();
    visionActive = true;
    freshVisionThisCycle = true;
  }

  private void readPassingVision(PerceptionProvider vision) {
    freshVisionThisCycle = false;
    visionActive = false;
    if (vision == null || !vision.isResultFresh()) {
      return;
    }
    double ts = vision.getResultTimestamp();
    if (ts == lastVisionTimestamp) {
      return;
    }
    lastVisionTimestamp = ts;

    if (!vision.isSeeingPassingTag()) {
      return;
    }
    visionTxDeg = vision.getYawToPassingTagDeg();
    distanceToHubMeters = vision.getDistanceToPassingTag();
    trackedTagId = vision.getTrackedPassingTagId();
    visionActive = true;
    freshVisionThisCycle = true;
  }
}
