package frc.robot.autonomy;

import edu.wpi.first.wpilibj.Timer;

/**
 * The collect↔shoot cycle latch with minimum dwells — the debounce that stops the behavior tree
 * from thrashing between collecting and shooting. A purely stateless tree would flip branches the
 * instant a condition wobbled; instead the world-facing "should we shoot?" signal is held here with
 * hysteresis: COLLECT is held for at least {@link AutonomyConstants#kCollectSeconds} (or until we
 * are positively full), and SHOOT for at least {@link AutonomyConstants#kMinShootSeconds} (and at
 * most {@link AutonomyConstants#kShootTimeoutSeconds} so a never-confirming shot can't hang the
 * cycle).
 *
 * <p>"Full" comes from {@link PossessionProvider} (always false today — no ball sensor), so the
 * collect dwell timer is what actually triggers the shoot phase. "Shot confirmed" is the scoring
 * status reaching SUCCEEDED (feed gate open) — an early-exit, not required.
 */
public final class MatchCycle {

  public enum Phase {
    COLLECT,
    SHOOT
  }

  private Phase phase = Phase.COLLECT;
  private final Timer timer = new Timer();

  public MatchCycle() {
    timer.restart();
  }

  public void reset() {
    phase = Phase.COLLECT;
    timer.restart();
  }

  /** Advance the latch one loop. {@code full}/{@code shootConfirmed} are the early-exit signals. */
  public void update(boolean full, boolean shootConfirmed) {
    switch (phase) {
      case COLLECT -> {
        if (full || timer.hasElapsed(AutonomyConstants.kCollectSeconds)) {
          phase = Phase.SHOOT;
          timer.restart();
        }
      }
      case SHOOT -> {
        boolean minDwellMet = timer.hasElapsed(AutonomyConstants.kMinShootSeconds);
        boolean timedOut = timer.hasElapsed(AutonomyConstants.kShootTimeoutSeconds);
        if ((minDwellMet && shootConfirmed) || timedOut) {
          phase = Phase.COLLECT;
          timer.restart();
        }
      }
    }
  }

  public boolean inShootPhase() {
    return phase == Phase.SHOOT;
  }

  public Phase phase() {
    return phase;
  }
}
