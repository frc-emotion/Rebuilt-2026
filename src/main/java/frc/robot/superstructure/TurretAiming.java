package frc.robot.superstructure;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.epilogue.Logged;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

/**
 * The aiming brain, ported verbatim from the legacy TurretAutoAimCommand: pure visual servoing
 * (target = current position + camera tx) carried between frames by gyro-delta feedforward.
 * Owned by the Superstructure; the Turret subsystem knows nothing about vision or the gyro.
 */
@Logged
public class TurretAiming {
  private static final double kAimedDeadbandDeg = 3.0;
  private static final boolean kGyroFfEnabled = true;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double targetPositionRot = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double visionTxDeg = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean visionActive = false;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double distanceToHubMeters = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private int trackedTagId = -1;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean passingDirectionLocked = false;

  @Logged(importance = Logged.Importance.DEBUG)
  private double gyroFeedforwardRot = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
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

  /** Hub tracking, one loop: gyro FF + fresh-frame visual servo + wrap-fed-back command (W2). */
  public void trackHub(Turret turret, Vision vision, double yawDeg, double omegaRadPerSec) {
    this.omegaRadPerSec = omegaRadPerSec; // sampled + logged; compensation deliberately off (W4)
    applyGyroFf(yawDeg);
    readHubVision(vision);
    if (freshVisionThisCycle) {
      targetPositionRot = turret.getPositionRot() + visionTxDeg / 360.0;
      seenFreshHubFrameSinceReset = true;
    }
    targetPositionRot = turret.setTargetPosition(Rotations.of(targetPositionRot)).in(Rotations);
  }

  /** Passing tracking: same loop shape, passing-tag vision, direction latch (legacy semantics). */
  public void trackPassing(Turret turret, Vision vision, double yawDeg) {
    applyGyroFf(yawDeg);
    readPassingVision(vision);
    if (freshVisionThisCycle) {
      targetPositionRot = turret.getPositionRot() + visionTxDeg / 360.0;
      passingDirectionLocked = true;
    }
    targetPositionRot = turret.setTargetPosition(Rotations.of(targetPositionRot)).in(Rotations);
  }

  /** Manual rebase (W7): the closed-loop target follows the actual position during manual jog. */
  public void rebaseToCurrent(double currentTurretPositionRot) {
    targetPositionRot = currentTurretPositionRot;
  }

  /** Aim gate: last vision tx < 3° AND at least one fresh hub frame since reset (W10 fix). */
  public boolean isAimed() {
    return seenFreshHubFrameSinceReset && Math.abs(visionTxDeg) < kAimedDeadbandDeg;
  }

  /** Last fresh-frame hub distance — held between frames (legacy command semantics). */
  public double getDistanceToHub() {
    return distanceToHubMeters;
  }

  public boolean isPassingLocked() {
    return passingDirectionLocked;
  }

  // Integrate measured yaw DELTAS into the setpoint so the turret stays field-pointed between
  // vision frames. The ADDITION (not subtraction) encodes the turret/gyro sign convention and is
  // load-bearing (W3). A rate-based version with a 1.75 lead fudge was tried and abandoned.
  private void applyGyroFf(double currentYawDeg) {
    gyroFeedforwardRot = 0.0;
    if (!kGyroFfEnabled) {
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
  private void readHubVision(Vision vision) {
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

  private void readPassingVision(Vision vision) {
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
