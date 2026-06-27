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

  public DriveToNode(Navigator navigator, Supplier<Pose2d> target) {
    this.navigator = navigator;
    this.target = target;
  }

  @Override
  public Status tick() {
    // The navigator dedupes a repeated target, so it is safe to (re)issue every tick — no stale
    // per-node cache to get out of sync across cycles.
    navigator.goTo(target.get());
    return navigator.atTarget() ? Status.SUCCESS : Status.RUNNING;
  }
}
