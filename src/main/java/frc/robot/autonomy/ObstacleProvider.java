package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * SEAM: dynamic field obstacles — primarily other robots — for the navigator to route around. The
 * centers returned here are pushed into PathPlanner's {@code setDynamicObstacles} each loop by
 * {@link PathPlannerNavigator} (inflated to keep-out boxes), so anything listed makes the planner
 * path around it. This robot has no opponent perception yet (single turret AprilTag camera); {@link
 * #NONE} is the production default, and {@link SimOpponentProvider} is a dashboard-driven stand-in
 * for proving avoidance in sim. When real detection lands (problems.md #4) it implements this
 * interface and nothing else changes — not the navigator, not the behavior tree.
 */
public interface ObstacleProvider {

  /** Detected robot CENTER positions to avoid this loop (blue-origin meters). Empty = nothing. */
  List<Translation2d> dynamicObstacles();

  /** The no-op default: nothing to avoid. */
  ObstacleProvider NONE = List::of;
}
