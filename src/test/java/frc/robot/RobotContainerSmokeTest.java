package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.runtime.SkillInterpreter.AxisStatus;
import frc.robot.runtime.reflex.ScoringSequencer.Phase;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * End-to-end wiring smoke test for the skill-server robot: the WHOLE robot constructs in sim
 * (drive, the generic mechanism layer from mechanisms.json, vision sim, the interpreter from
 * skills.json, the NT skill server, the local driver, PathPlanner autos), the scheduler runs, and
 * the runtime rests at IDLE. Per-skill behavior equivalence is covered by EquivalenceHarnessTest;
 * this is wiring only.
 */
class RobotContainerSmokeTest {
  private static RobotContainer container;

  @BeforeAll
  static void setup() {
    HAL.initialize(500, 0);
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
    assertNotNull(container.runtime, "the runtime subsystem must exist");
    assertNotNull(container.mechanisms, "the mechanism layer must exist");
    assertTrue(container.mechanisms.isValid(), "mechanisms.json must load valid");
    assertNotNull(container.getAutonomousCommand(), "auto chooser must yield a default command");
    // The exact legacy named-command strings the deploy .auto files reference:
    for (String name :
        new String[] {
          "intakeOut", "intakeIn", "shoot", "stopAll", "autoShoot", "feedIndexers", "reverseIndexer"
        }) {
      assertTrue(NamedCommands.hasCommand(name), "named command missing: " + name);
    }
  }

  @Test
  void schedulerRunsAndRestsAtIdle() {
    container.getRuntime().onEnable();
    runScheduler(25);
    assertEquals(Phase.IDLE, container.getRuntime().interpreter().phase());
    assertEquals(AxisStatus.OK, container.getRuntime().interpreter().scoringStatus());
  }

  @Test
  void manualModeReachableAndExitsToIdle() {
    container.getRuntime().onEnable();
    runScheduler(5);
    container.getRuntime().server().setManualMode(true);
    runScheduler(5);
    assertEquals(Phase.MANUAL, container.getRuntime().interpreter().phase());
    container.getRuntime().server().setManualMode(false);
    runScheduler(5);
    assertEquals(Phase.IDLE, container.getRuntime().interpreter().phase());
  }

  @Test
  void intakeToggleReachesIntakingPhase() {
    container.getRuntime().onEnable();
    container.getRuntime().server().setIntakeDeploy(true);
    runScheduler(400); // arm sim needs time to swing past the 5-degree isOut threshold
    assertEquals(Phase.INTAKING, container.getRuntime().interpreter().phase());
    container.getRuntime().server().setIntakeDeploy(false);
    runScheduler(400);
    assertEquals(Phase.IDLE, container.getRuntime().interpreter().phase());
  }
}
