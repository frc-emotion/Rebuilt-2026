package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * SEAM (empty today): dynamic field obstacles — primarily other robots — for the navigator to route
 * around. This version has NO opponent perception (single turret AprilTag camera; no depth/object
 * detection), so {@link #NONE} returns an empty list and {@link PathPlannerNavigator} only avoids
 * the static navgrid. When opponent detection lands (problems.md #4), feed live obstacle positions
 * here and have the navigator inject them into PathPlanner's dynamic obstacle set; the behavior
 * tree does not change.
 */
public interface ObstacleProvider {

  /** Field positions to avoid this loop. Empty until opponent perception exists. */
  List<Translation2d> dynamicObstacles();

  /** The no-op default: nothing to avoid. */
  ObstacleProvider NONE = List::of;
}
