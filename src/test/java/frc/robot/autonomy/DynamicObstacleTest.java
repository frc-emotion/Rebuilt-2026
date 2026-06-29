package frc.robot.autonomy;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Diagnostic: does {@code Pathfinding.setDynamicObstacles} actually make PathPlanner route AROUND a
 * box (isolated from the swerve sim / the BT)? Plans straight up the open neutral zone with a box
 * planted on the straight line and checks the resulting path bends clear of it.
 */
class DynamicObstacleTest {

  @BeforeAll
  static void once() {
    HAL.initialize(500, 0);
  }

  @Test
  void pathfinderRoutesAroundADynamicObstacle() throws InterruptedException {
    Pathfinding.ensureInitialized();
    Translation2d start = new Translation2d(6.0, 2.0);
    Translation2d goal = new Translation2d(6.0, 6.0);
    Translation2d obs = new Translation2d(6.0, 4.0); // dead on the straight line
    double h = AutonomyConstants.kObstacleClearanceMeters;

    Pathfinding.setDynamicObstacles(
        List.of(
            Pair.of(
                new Translation2d(obs.getX() - h, obs.getY() - h),
                new Translation2d(obs.getX() + h, obs.getY() + h))),
        start);
    Pathfinding.setStartPosition(start);
    Pathfinding.setGoalPosition(goal);

    // The planner runs on a background thread; drain ~4 s, keeping the latest path.
    PathPlannerPath path = null;
    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, Math.toRadians(540), Math.toRadians(720));
    for (int i = 0; i < 80; i++) {
      if (Pathfinding.isNewPathAvailable()) {
        path = Pathfinding.getCurrentPath(constraints, new GoalEndState(0.0, Rotation2d.kZero));
      }
      Thread.sleep(50);
    }

    assertNotNull(path, "pathfinder must produce a path");
    List<Pose2d> poses = path.getPathPoses();
    double half = h - 0.2; // shrink slightly so we only flag a real intrusion
    boolean entersBox =
        poses.stream()
            .anyMatch(
                p ->
                    Math.abs(p.getX() - obs.getX()) < half
                        && Math.abs(p.getY() - obs.getY()) < half);
    System.out.println(
        "[DYN-OBS] path points="
            + poses.size()
            + " entersBox="
            + entersBox
            + " first="
            + poses.get(0)
            + " last="
            + poses.get(poses.size() - 1));
    assertFalse(entersBox, "path must route AROUND the dynamic obstacle, not through it");
  }
}
