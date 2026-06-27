package frc.robot.autonomy.bt;

/**
 * Sequence: ticks its children in order and stops at the first that is not yet SUCCEEDED. Returns
 * RUNNING while a child is working, FAILURE if a child fails, SUCCESS only when every child has
 * succeeded. Used for "do A, then B, then C" — e.g. drive to the shoot pose, THEN shoot.
 *
 * <p>Reactive: re-ticks from the first child each loop. Earlier children are expected to be cheap
 * and idempotent once satisfied (e.g. a DriveTo returns SUCCESS immediately while already at its
 * target), so the sequence flows forward without extra bookkeeping.
 */
public final class Sequence implements Node {
  private final Node[] children;

  public Sequence(Node... children) {
    this.children = children;
  }

  @Override
  public Status tick() {
    for (Node child : children) {
      Status status = child.tick();
      if (status != Status.SUCCESS) {
        return status; // RUNNING or FAILURE stops the sequence here
      }
    }
    return Status.SUCCESS;
  }

  @Override
  public void reset() {
    for (Node child : children) {
      child.reset();
    }
  }
}
