package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.superstructure.RobotState;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * End-to-end wiring smoke test: the WHOLE new robot constructs in sim (drive, all mechanism sim
 * IOs, vision sim, superstructure, bindings, PathPlanner autos), the scheduler runs, and manual
 * mode is reachable and exits to IDLE. Full goal-flow demonstration: ./gradlew simulateJava.
 */
class RobotContainerSmokeTest {
  private static RobotContainer container;

  @BeforeAll
  static void setup() {
    HAL.initialize(500, 0);
    // Drop any subsystems registered by other test classes in this JVM before we construct ours.
    CommandScheduler.getInstance().unregisterAllSubsystems();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    container = new RobotContainer();
  }

  @AfterAll
  static void teardown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  private static void runScheduler(int ticks) {
    for (int i = 0; i < ticks; i++) {
      CommandScheduler.getInstance().run();
    }
  }

  @Test
  void constructsAndRegistersEverything() {
    assertNotNull(container.superstructure, "all feature flags are on; superstructure must exist");
    assertNotNull(container.getAutonomousCommand(), "auto chooser must yield a default command");
    // The exact legacy named-command strings the deploy .auto files reference:
    for (String name :
        new String[] {"intakeOut", "intakeIn", "shoot", "stopAll", "autoShoot", "feedIndexers",
          "reverseIndexer"}) {
      assertTrue(NamedCommands.hasCommand(name), "named command missing: " + name);
    }
  }

  @Test
  void schedulerRunsAndRestsAtIdle() {
    container.getSuperstructure().onEnable();
    runScheduler(25);
    assertEquals(RobotState.IDLE, container.getSuperstructure().getState());
  }

  @Test
  void manualModeEntersFromAnyStateAndExitsToIdle() {
    container.getSuperstructure().onEnable();
    runScheduler(5);
    container.getSuperstructure().toggleManualMode();
    runScheduler(5);
    assertEquals(RobotState.MANUAL, container.getSuperstructure().getState());
    container.getSuperstructure().toggleManualMode();
    runScheduler(5);
    assertEquals(RobotState.IDLE, container.getSuperstructure().getState());
  }

  @Test
  void intakeToggleReachesIntakingState() {
    container.getSuperstructure().onEnable();
    container.getSuperstructure().setIntakeRequested(true);
    runScheduler(400); // arm sim needs time to swing past the 5-degree isOut threshold
    assertEquals(RobotState.INTAKING, container.getSuperstructure().getState());
    container.getSuperstructure().setIntakeRequested(false);
    runScheduler(400);
    assertEquals(RobotState.IDLE, container.getSuperstructure().getState());
  }

  @Test
  void shootWithoutVisionStaysSpinningUpAndNeverFeeds() {
    // The W12 safety end-to-end: no camera frames -> never aimed -> feed gate never opens.
    container.getSuperstructure().onEnable();
    edu.wpi.first.wpilibj2.command.Command hold =
        container.getSuperstructure().goalCommand(frc.robot.superstructure.Goal.SHOOT);
    hold.schedule();
    runScheduler(300);
    assertEquals(RobotState.SPINNING_UP, container.getSuperstructure().getState());
    hold.cancel();
    runScheduler(5);
    assertEquals(RobotState.IDLE, container.getSuperstructure().getState());
  }
}
