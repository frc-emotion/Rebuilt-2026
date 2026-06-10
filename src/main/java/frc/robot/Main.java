package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Entry point. Cutover complete: the new robot is live; frc.robot.legacy is archived. */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
