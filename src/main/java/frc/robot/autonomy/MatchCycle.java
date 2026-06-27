package frc.robot.autonomy;

import edu.wpi.first.wpilibj.Timer;

/**
 * The collect↔shoot cycle latch with ARRIVAL-BASED minimum dwells — the debounce that stops the
 * behavior tree from thrashing between collecting and shooting. A purely stateless tree would flip
 * branches the instant a condition wobbled; a naive in-phase timer would flip before the robot even
 * finished driving (the v1 bug). So the dwell only counts while the robot is actually PARKED at the
 * current phase's pose:
 *
 * <ul>
 *   <li>COLLECT → SHOOT once parked at the collection region for {@link
 *       AutonomyConstants#kCollectSeconds} (or positively full).
 *   <li>SHOOT → COLLECT once parked at the shoot pose and either firing for {@link
 *       AutonomyConstants#kMinShootSeconds} with the shot confirmed, or {@link
 *       AutonomyConstants#kShootMaxSeconds} have passed at the pose (give up this shot).
 * </ul>
 *
 * <p>A per-phase HARD cap ({@link AutonomyConstants#kPhaseHardTimeoutSeconds}) guarantees forward
 * progress even if a pose can never be reached (blocked), so the cycle can never hang. "Full" comes
 * from {@link PossessionProvider} (always false — no ball sensor); "shot confirmed" is the scoring
 * status reaching SUCCEEDED (feed gate open).
 */
public final class MatchCycle {

  public enum Phase {
    COLLECT,
    SHOOT
  }

  private Phase phase = Phase.COLLECT;
  private final Timer phaseTimer = new Timer(); // time since entering the phase (hard cap)
  private final Timer parkedTimer = new Timer(); // time parked at the phase's pose (the dwell)

  public MatchCycle() {
    restartPhase();
  }

  public void reset() {
    phase = Phase.COLLECT;
    restartPhase();
  }

  /**
   * Advance the latch one loop.
   *
   * @param full possession says we are holding (false today — no sensor)
   * @param atTarget the robot is parked at the current phase's pose
   * @param shootConfirmed the scoring status has reached SUCCEEDED (feed gate open)
   */
  public void update(boolean full, boolean atTarget, boolean shootConfirmed) {
    if (!atTarget) {
      parkedTimer.restart(); // dwell only accrues while actually parked
    }
    boolean hardCap = phaseTimer.hasElapsed(AutonomyConstants.kPhaseHardTimeoutSeconds);

    switch (phase) {
      case COLLECT -> {
        boolean dwellDone = atTarget && parkedTimer.hasElapsed(AutonomyConstants.kCollectSeconds);
        if (full || dwellDone || hardCap) {
          phase = Phase.SHOOT;
          restartPhase();
        }
      }
      case SHOOT -> {
        boolean minCommitMet =
            atTarget && parkedTimer.hasElapsed(AutonomyConstants.kMinShootSeconds);
        boolean shotWindowDone =
            atTarget && parkedTimer.hasElapsed(AutonomyConstants.kShootMaxSeconds);
        if ((minCommitMet && shootConfirmed) || shotWindowDone || hardCap) {
          phase = Phase.COLLECT;
          restartPhase();
        }
      }
    }
  }

  private void restartPhase() {
    phaseTimer.restart();
    parkedTimer.restart();
  }

  public boolean inShootPhase() {
    return phase == Phase.SHOOT;
  }

  public Phase phase() {
    return phase;
  }
}
