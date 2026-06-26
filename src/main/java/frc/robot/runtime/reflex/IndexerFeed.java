package frc.robot.runtime.reflex;

import frc.robot.runtime.Mechanisms;

/**
 * The four indexer feed patterns, ported VERBATIM from the legacy Superstructure state behaviors.
 * Each writes all three stages (vertical, horizontal, upward) to the mechanism layer. The legacy
 * stage enum order (VERTICAL, HORIZONTAL, UPWARD) and speeds are preserved exactly.
 */
public final class IndexerFeed {
  public static final String VERTICAL = "indexerVertical";
  public static final String HORIZONTAL = "indexerHorizontal";
  public static final String UPWARD = "indexerUpward";

  private IndexerFeed() {}

  /**
   * Resting feed (W18, verbatim): vertical 26.25 RPS while the intake is out, or full 35 while the
   * operator holds LT (manual feed); otherwise stopped. Horizontal + upward always stopped.
   */
  public static void rest(Mechanisms m, boolean manualFeedHeld, boolean intakeOut) {
    if (manualFeedHeld) {
      m.setVelocity(VERTICAL, ReflexConstants.kVerticalSpeedRps);
    } else if (intakeOut) {
      m.setVelocity(VERTICAL, ReflexConstants.kIntakingVerticalSpeedRps);
    } else {
      m.stop(VERTICAL);
    }
    m.stop(HORIZONTAL);
    m.stop(UPWARD);
  }

  /**
   * Spin-up / fire feed: vertical always at 35; horizontal 35 + upward 100 only when the gate is
   * open. Closed gate reproduces SPINNING_UP / PASS_SPINNING_UP; open gate reproduces SHOOTING /
   * PASSING.
   */
  public static void gated(Mechanisms m, boolean gateOpen) {
    m.setVelocity(VERTICAL, ReflexConstants.kVerticalSpeedRps);
    if (gateOpen) {
      m.setVelocity(HORIZONTAL, ReflexConstants.kHorizontalSpeedRps);
      m.setVelocity(UPWARD, ReflexConstants.kUpwardSpeedRps);
    } else {
      m.stop(HORIZONTAL);
      m.stop(UPWARD);
    }
  }

  /** CLEARING: every stage backward at FULL speed (team decision, atomic back-off). */
  public static void clearing(Mechanisms m) {
    m.setVelocity(HORIZONTAL, -ReflexConstants.kHorizontalSpeedRps);
    m.setVelocity(VERTICAL, -ReflexConstants.kVerticalSpeedRps);
    m.setVelocity(UPWARD, -ReflexConstants.kUpwardSpeedRps);
  }

  /**
   * UNJAM (W19): every stage backward at half speed (the shooter runs forward at max elsewhere).
   */
  public static void unjam(Mechanisms m) {
    m.setVelocity(
        HORIZONTAL, -ReflexConstants.kHorizontalSpeedRps * ReflexConstants.kUnjamFraction);
    m.setVelocity(VERTICAL, -ReflexConstants.kVerticalSpeedRps * ReflexConstants.kUnjamFraction);
    m.setVelocity(UPWARD, -ReflexConstants.kUpwardSpeedRps * ReflexConstants.kUnjamFraction);
  }
}
