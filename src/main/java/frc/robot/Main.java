package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Entry point. Points at the LEGACY robot until the Phase 4 cutover so the robot stays deployable
 * throughout the refactor. The new robot lives in frc.robot and is built alongside.
 */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(frc.robot.legacy.Robot::new);
  }
}
