package frc.robot.autonomy.bt;

/**
 * Fallback (a.k.a. selector / priority node): tries its children in order each tick and returns the
 * first one that is NOT failing. This is the priority structure of the match tree — higher-priority
 * branches (safety, endgame) get first refusal every loop, and only if they decline (FAILURE) does
 * control fall through to the next.
 *
 * <p>Reactive: it re-evaluates from the top every tick, so a higher-priority branch can preempt a
 * lower one the instant its guard becomes true.
 */
public final class Fallback implements Node {
  private final Node[] children;

  public Fallback(Node... children) {
    this.children = children;
  }

  @Override
  public Status tick() {
    for (Node child : children) {
      Status status = child.tick();
      if (status != Status.FAILURE) {
        return status; // SUCCESS or RUNNING wins; stop here
      }
    }
    return Status.FAILURE;
  }

  @Override
  public void reset() {
    for (Node child : children) {
      child.reset();
    }
  }
}
