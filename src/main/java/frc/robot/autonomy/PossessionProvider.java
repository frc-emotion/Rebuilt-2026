package frc.robot.autonomy;

/**
 * SEAM (no sensor today): how many game pieces the robot is holding. This robot has NO possession
 * sensing (the indexer has no beam-break / ball count — problems.md #6), so {@link #UNKNOWN}
 * reports UNKNOWN and {@link #isFull()} is always false. The match tree therefore falls back to a
 * collection-DWELL timer instead of a "we are full" signal. When beam-breaks land, implement this
 * from the count and the tree's "full OR dwell-expired" guard starts using the real signal.
 */
public interface PossessionProvider {

  enum Possession {
    EMPTY,
    HOLDING,
    UNKNOWN
  }

  Possession state();

  /** True only when we positively know the robot is full. False today (no sensor). */
  default boolean isFull() {
    return state() == Possession.HOLDING;
  }

  /** The no-op default: we cannot tell. */
  PossessionProvider UNKNOWN = () -> Possession.UNKNOWN;
}
