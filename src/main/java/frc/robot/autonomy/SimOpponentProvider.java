package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

/**
 * A SIM-only {@link ObstacleProvider}: a fake "opponent robot" you can place and move from the
 * dashboard to prove out dynamic obstacle avoidance before real robot detection exists. It is the
 * stand-in that the future detector will replace — the navigator pushes whatever this returns into
 * PathPlanner's dynamic obstacles, so our robot routes around it.
 *
 * <p>Dashboard controls (flat keys, next to {@code TeleopAutonomy}):
 *
 * <ul>
 *   <li>{@code SimOpponentEnabled} (bool) — place the opponent on the field.
 *   <li>{@code SimOpponentX} / {@code SimOpponentY} (m, blue-origin) — where it sits.
 *   <li>{@code SimOpponentOscillate} (bool) — sweep it laterally so you can watch live re-routing.
 * </ul>
 *
 * <p>Its pose is published to the {@code Pose} table as {@code SimOpponent}, so it shows up ON THE
 * SAME Field2d widget as the robot (same table the robot pose uses). Disabled → off-field + no
 * obstacle.
 */
public final class SimOpponentProvider implements ObstacleProvider {
  private static final double[] kOffField = {-2.0, -2.0, 0.0};
  private static final double kOscillateRateRadPerSec = 0.8;
  private static final double kOscillateAmplitudeMeters = 1.5;

  private final DoubleArrayPublisher posePub =
      NetworkTableInstance.getDefault()
          .getTable("Pose")
          .getDoubleArrayTopic("SimOpponent")
          .publish();
  private final Timer clock = new Timer();

  public SimOpponentProvider() {
    SmartDashboard.putBoolean("SimOpponentEnabled", false);
    SmartDashboard.putNumber("SimOpponentX", 3.0); // our (blue) half, in front of the robot
    SmartDashboard.putNumber("SimOpponentY", 4.0);
    SmartDashboard.putBoolean("SimOpponentOscillate", false);
    posePub.set(kOffField);
    clock.start();
  }

  @Override
  public List<Translation2d> dynamicObstacles() {
    if (!SmartDashboard.getBoolean("SimOpponentEnabled", false)) {
      posePub.set(kOffField);
      return List.of();
    }
    double x = SmartDashboard.getNumber("SimOpponentX", 3.0);
    double y = SmartDashboard.getNumber("SimOpponentY", 4.0);
    if (SmartDashboard.getBoolean("SimOpponentOscillate", false)) {
      y += Math.sin(clock.get() * kOscillateRateRadPerSec) * kOscillateAmplitudeMeters;
    }
    posePub.set(new double[] {x, y, 0.0});
    return List.of(new Translation2d(x, y));
  }
}
