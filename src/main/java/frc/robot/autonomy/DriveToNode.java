package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autonomy.bt.Node;
import frc.robot.autonomy.bt.Status;
import java.util.function.Supplier;

/**
 * A behavior-tree leaf that drives the robot to a field pose via the {@link Navigator}. The target
 * is a supplier so it can move (e.g. the collection sweep oscillates its target). Returns RUNNING
 * while traveling and SUCCESS once within tolerance; re-issues the route only when the target
 * actually changes, so re-ticking is cheap and idempotent once parked.
 */
public final class DriveToNode implements Node {
  private final Navigator navigator;
  private final Supplier<Pose2d> target;
  private Pose2d committed;

  public DriveToNode(Navigator navigator, Supplier<Pose2d> target) {
    this.navigator = navigator;
    this.target = target;
  }

  @Override
  public Status tick() {
    Pose2d desired = target.get();
    if (committed == null || !committed.equals(desired)) {
      navigator.goTo(desired);
      committed = desired;
    }
    return navigator.atTarget() ? Status.SUCCESS : Status.RUNNING;
  }

  @Override
  public void reset() {
    committed = null;
  }
}
