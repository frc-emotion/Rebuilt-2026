package frc.robot.autonomy;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;

/**
 * The real {@link Navigator}: PathPlanner runtime pathfinding over the deployed navgrid. {@link
 * #goTo} schedules a fresh {@code pathfindToPoseFlipped} command (alliance-flipped) from the
 * current pose to the target; arrival is judged by pose tolerance. Static-field obstacle avoidance
 * comes from the navgrid for free; DYNAMIC obstacle avoidance (opponents) is fed each loop from the
 * {@link ObstacleProvider} seam into PathPlanner's {@code setDynamicObstacles}, so a detected (or
 * simulated) robot in the way makes the planner route around it — no behavior-tree change.
 */
public final class PathPlannerNavigator implements Navigator {
  private final Drive drive;
  private final PathConstraints constraints;
  private final double translationToleranceMeters;
  private final ObstacleProvider obstacles;

  private Command active;
  private Pose2d target;
  private List<Translation2d> lastObstacles = List.of();

  public PathPlannerNavigator(
      Drive drive,
      PathConstraints constraints,
      double translationToleranceMeters,
      ObstacleProvider obstacles) {
    this.drive = drive;
    this.constraints = constraints;
    this.translationToleranceMeters = translationToleranceMeters;
    this.obstacles = obstacles;
  }

  @Override
  public void goTo(Pose2d requested) {
    pushDynamicObstacles(); // refresh opponent keep-outs each loop (replan when they move)
    // If an opponent is sitting ON the requested point, PathPlanner can't path INTO an obstacle —
    // so divert to the nearest open spot in the region first (drive to a region, not a buried
    // point).
    Pose2d target =
        LegalRegion.nearestClear(
            requested, obstacles.dynamicObstacles(), AutonomyConstants.kObstacleClearanceMeters);
    // Idempotent: re-issuing the SAME target every loop must NOT restart pathfinding (that would
    // thrash the planner). Only (re)schedule when the target changes or the route finished/was
    // cancelled. This also removes any need for callers to cache a "committed" target.
    boolean sameTarget = target.equals(this.target);
    boolean stillRunning = active != null && active.isScheduled();
    if (sameTarget && (stillRunning || atTarget())) {
      // En route to, or already parked at, this target — don't respawn the planner. (Only re-drive
      // if the robot has DRIFTED beyond tolerance after the route finished.)
      return;
    }
    this.target = target;
    if (active != null) {
      active.cancel();
    }
    if (!AutoBuilder.isPathfindingConfigured()) {
      DriverStation.reportError("[Navigator] pathfinding not configured — cannot driveTo", false);
      active = null;
      return;
    }
    active = drive.driveToPose(target, constraints);
    CommandScheduler.getInstance().schedule(active);
  }

  @Override
  public boolean atTarget() {
    if (target == null) {
      return false;
    }
    double distance = drive.getPose().getTranslation().getDistance(target.getTranslation());
    return distance < translationToleranceMeters;
  }

  @Override
  public void stop() {
    if (active != null) {
      active.cancel();
      active = null;
    }
    target = null;
  }

  @Override
  public Pose2d pose() {
    return drive.getPose();
  }

  // Hand the obstacle keep-out boxes to PathPlanner. We push EVERY loop (so they persist even if
  // the
  // running pathfind command resets them), and additionally force the in-flight route to recompute
  // when the set CHANGES (otherwise an already-running command keeps its old straight path and
  // drives
  // through — the planner itself routes around them, proved by DynamicObstacleTest).
  private void pushDynamicObstacles() {
    if (!AutoBuilder.isPathfindingConfigured()) {
      return;
    }
    List<Translation2d> centers = obstacles.dynamicObstacles();
    boolean changed = obstaclesChanged(centers);
    if (centers.isEmpty() && lastObstacles.isEmpty() && !changed) {
      return; // nothing now, nothing before — don't touch the pathfinder at all
    }
    // The static pathfinder is created lazily (only when a pathfinding command first runs), so
    // pushing obstacles before that would NPE. Force it up first; ensureInitialized is idempotent.
    Pathfinding.ensureInitialized();
    double h = AutonomyConstants.kObstacleClearanceMeters;
    List<Pair<Translation2d, Translation2d>> boxes = new ArrayList<>(centers.size());
    for (Translation2d c : centers) {
      boxes.add(
          Pair.of(
              new Translation2d(c.getX() - h, c.getY() - h),
              new Translation2d(c.getX() + h, c.getY() + h)));
    }
    Pathfinding.setDynamicObstacles(boxes, drive.getPose().getTranslation());
    if (changed) {
      lastObstacles = centers;
      if (active != null) {
        active.cancel();
        active = null;
      }
      target = null; // the rest of this goTo() reschedules a fresh path that avoids them
    }
  }

  private boolean obstaclesChanged(List<Translation2d> now) {
    if (now.size() != lastObstacles.size()) {
      return true;
    }
    for (int i = 0; i < now.size(); i++) {
      if (now.get(i).getDistance(lastObstacles.get(i)) > 0.1) {
        return true;
      }
    }
    return false;
  }
}
