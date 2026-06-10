package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** The new robot. Main points here after the Phase 4 cutover. */
@Logged
public class Robot extends TimedRobot {
  // Flip to true for competition: Epilogue publishes only CRITICAL fields to NetworkTables
  // (everything still lands in the log files).
  public static final boolean MATCH_MODE = false;

  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  // Phoenix hoot replay (timestamps + joysticks) for deterministic log replay; no-op on a field.
  private final HootAutoReplay timeAndJoystickReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  public Robot() {
    Epilogue.configure(config -> {
      config.root = "Robot";
      config.minimumImportance =
          MATCH_MODE ? Logged.Importance.CRITICAL : Logged.Importance.DEBUG;
    });
    DataLogManager.start();
    SignalLogger.start();
    robotContainer = new RobotContainer();
    Epilogue.bind(this);
    System.out.println(
        "[TELEMETRY] MATCH_MODE=" + MATCH_MODE + " -> NT publishes "
            + (MATCH_MODE ? "CRITICAL only" : "DEBUG and up"));
  }

  @Override
  public void robotPeriodic() {
    timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    if (robotContainer.getSuperstructure() != null) {
      robotContainer.getSuperstructure().onEnable();
    }
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (robotContainer.getSuperstructure() != null) {
      robotContainer.getSuperstructure().onEnable();
    }
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
