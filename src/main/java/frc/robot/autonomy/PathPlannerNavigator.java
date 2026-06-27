package frc.robot.autonomy;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;

/**
 * The real {@link Navigator}: PathPlanner runtime pathfinding over the deployed navgrid. {@link
 * #goTo} schedules a fresh {@code pathfindToPoseFlipped} command (alliance-flipped) from the
 * current pose to the target; arrival is judged by pose tolerance. Static-field obstacle avoidance
 * comes from the navgrid for free; DYNAMIC obstacle avoidance (opponents) is not available yet —
 * that is the {@link ObstacleProvider} seam, no-op today.
 */
public final class PathPlannerNavigator implements Navigator {
  private final Drive drive;
  private final PathConstraints constraints;
  private final double translationToleranceMeters;

  private Command active;
  private Pose2d target;

  public PathPlannerNavigator(
      Drive drive, PathConstraints constraints, double translationToleranceMeters) {
    this.drive = drive;
    this.constraints = constraints;
    this.translationToleranceMeters = translationToleranceMeters;
  }

  @Override
  public void goTo(Pose2d target) {
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
}
