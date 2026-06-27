package frc.robot.runtime;

import frc.robot.runtime.reflex.ScoringSequencer.Phase;

/**
 * The named "done" predicate registry for the skill table (status instrumentation only). Each
 * scoring skill in {@code skills.json} names a {@code done} predicate; this maps that name to a
 * pure function of the current scoring {@link Phase}. The orchestration brain reads the result over
 * NetworkTables to decide whether a requested skill has reached its objective — but the interpreter
 * NEVER auto-cancels a skill on done. Held skills stay held; done is purely advisory.
 *
 * <p>IMPORTANT: this robot has NO game-piece sensor (the indexer has no ball count), so {@code
 * feeding} is the best available proxy for "a shot happened": it is true once the feed gate has
 * opened and the robot is actively feeding/firing (phase SHOOTING or PASSING). It cannot confirm a
 * ball physically launched.
 */
public final class DonePredicates {
  private DonePredicates() {}

  /** Evaluate a named done predicate against the current phase. Unknown names are never done. */
  public static boolean evaluate(String name, Phase phase) {
    return switch (name == null ? "never" : name) {
      // Feed-gate-open proxy for a completed shot (no game-piece sensor exists on this robot).
      case "feeding" -> phase == Phase.SHOOTING || phase == Phase.PASSING;
      case "never" -> false;
      default -> false;
    };
  }
}
