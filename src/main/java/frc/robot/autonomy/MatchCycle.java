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
 *   <li>SHOOT → COLLECT once the feed gate has been OPEN (firing) for {@link
 *       AutonomyConstants#kShootEmptySeconds} — long enough to empty the hopper (no ball sensor, so
 *       we drain by time). Spin-up/aim time before the gate opens does not count toward the empty.
 * </ul>
 *
 * <p>A per-phase HARD cap ({@link AutonomyConstants#kPhaseHardTimeoutSeconds}) guarantees forward
 * progress even if a pose can never be reached (blocked) or we can never aim, so the cycle can
 * never hang. "Full" comes from {@link PossessionProvider} (always false — no ball sensor);
 * "feeding" is the scoring status reaching SUCCEEDED (feed gate open).
 */
public final class MatchCycle {

  public enum Phase {
    COLLECT,
    SHOOT
  }

  private Phase phase = Phase.COLLECT;
  private final Timer phaseTimer = new Timer(); // time since entering the phase (hard cap)
  private final Timer parkedTimer = new Timer(); // time parked at the phase's pose (the dwell)
  private final Timer feedingTimer = new Timer(); // time the feed gate has been open (the empty)

  public MatchCycle() {
    restartPhase();
  }

  public void reset() {
    phase = Phase.COLLECT;
    restartPhase();
  }

  /**
   * Jump straight into the SHOOT phase. Used when our HUB re-activates after an off-shift harvest —
   * the robot has already staged home loaded, so it should fire immediately rather than burn the
   * collect dwell again.
   */
  public void startShootPhase() {
    phase = Phase.SHOOT;
    restartPhase();
  }

  /**
   * Advance the latch one loop.
   *
   * @param full possession says we are holding (false today — no sensor)
   * @param atTarget the robot is parked at the current phase's pose
   * @param feeding the feed gate is open (scoring SUCCEEDED) — i.e. balls are leaving the hopper
   */
  public void update(boolean full, boolean atTarget, boolean feeding) {
    if (!atTarget) {
      parkedTimer.restart(); // dwell only accrues while actually parked
    }
    if (!feeding) {
      feedingTimer.restart(); // the empty only accrues while the gate is actually open
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
        boolean emptied = feedingTimer.hasElapsed(AutonomyConstants.kShootEmptySeconds);
        if (emptied || hardCap) {
          phase = Phase.COLLECT;
          restartPhase();
        }
      }
    }
  }

  private void restartPhase() {
    phaseTimer.restart();
    parkedTimer.restart();
    feedingTimer.restart();
  }

  public boolean inShootPhase() {
    return phase == Phase.SHOOT;
  }

  public Phase phase() {
    return phase;
  }
}
