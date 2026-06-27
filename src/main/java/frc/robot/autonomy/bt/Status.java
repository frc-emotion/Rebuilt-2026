package frc.robot.autonomy.bt;

/** The result of ticking a behavior-tree node once. Standard BT semantics. */
public enum Status {
  /** The node finished its job this tick. */
  SUCCESS,
  /** The node cannot make progress (a guard failed, or it gave up). */
  FAILURE,
  /** The node is still working; tick it again next loop. */
  RUNNING
}
