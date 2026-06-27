package frc.robot.autonomy.bt;

/**
 * A behavior-tree node. The whole tree is ticked once per robot loop from the root; each node
 * returns {@link Status}. This is a deliberately tiny, readable framework — a student should be
 * able to read the whole {@code bt/} package in a few minutes.
 */
public interface Node {

  /** Run this node for one loop and report what happened. */
  Status tick();

  /** Optional: clear any internal progress when the node is abandoned mid-run. */
  default void reset() {}
}
