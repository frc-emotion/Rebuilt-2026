package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * SEAM (empty today): a learned local-navigation cost term layered over PathPlanner's planner — the
 * place a future model would say "this patch of field is expensive right now" (congestion, a lane
 * an opponent tends to cut through, etc.). NO model is implemented and none should be: it has no
 * inputs yet (no opponent/field perception). {@link #ZERO} adds no cost, so the navigator plans on
 * geometry alone. When the inputs exist, an implementation returns an extra traversal cost per
 * field point and the navigator blends it into the pathfinder's edge weights — the tree is
 * unaffected.
 */
public interface NavCostProvider {

  /** Extra traversal cost at a field point (0 = no preference). */
  double extraCost(Translation2d point);

  /** The no-op default: geometry-only planning. */
  NavCostProvider ZERO = point -> 0.0;
}
