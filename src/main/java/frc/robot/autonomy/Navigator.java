package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * The movement primitive the behavior tree drives through. Abstracting "go to this field pose"
 * behind an interface is a deliberate improvement over wiring PathPlanner straight into the tree:
 * it makes the decision layer unit-testable with a fake, and it is the seam where the future
 * opponent-obstacle input and the learned local-nav cost term will be injected (they reshape the
 * route a Navigator produces, not the tree's decisions).
 *
 * <p>The real implementation ({@link PathPlannerNavigator}) uses PathPlanner runtime pathfinding
 * (navgrid-aware {@code pathfindToPoseFlipped}) — each {@link #goTo} computes a fresh route from
 * the current pose, so the tree issues many small dynamic paths, one per decision, never one big
 * path.
 */
public interface Navigator {

  /** (Re)start driving toward {@code target} (blue-origin field pose; alliance flip is handled). */
  void goTo(Pose2d target);

  /** True once the robot is within tolerance of the most recent {@link #goTo} target. */
  boolean atTarget();

  /** Cancel any active route and hold. */
  void stop();

  /** Current best-known field pose (odometry/vision-fused; ground truth in sim). */
  Pose2d pose();
}
