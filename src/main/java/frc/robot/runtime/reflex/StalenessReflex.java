package frc.robot.runtime.reflex;

import edu.wpi.first.wpilibj.Timer;

/**
 * Staleness → safe (NEW reflex, not a ported behavior). Tracks wall-clock time since the perception
 * pipeline last delivered a fresh frame; if that exceeds {@link
 * ReflexConstants#kPerceptionStaleSeconds} the interpreter treats vision-dependent skills as unsafe
 * and backs the shooter + feed off. The threshold is deliberately generous so this is INERT in
 * normal operation and during the equivalence harness (which delivers a fresh frame every loop —
 * "fresh" keys on frame arrival, not on whether a tag is present), and fires only on a genuinely
 * dead camera/coprocessor link. This is the one place the migration adds behavior the legacy robot
 * did not have; it is flagged as such in the final report.
 */
public final class StalenessReflex {
  private final Timer sinceFresh = new Timer();

  public StalenessReflex() {
    sinceFresh.start();
  }

  /** Re-seed on enable so we never start "stale". */
  public void reset() {
    sinceFresh.restart();
  }

  /** Call once per loop with whether perception produced a fresh frame this loop. */
  public void update(boolean perceptionFresh) {
    if (perceptionFresh) {
      sinceFresh.restart();
    }
  }

  public boolean isStale() {
    return sinceFresh.hasElapsed(ReflexConstants.kPerceptionStaleSeconds);
  }
}
