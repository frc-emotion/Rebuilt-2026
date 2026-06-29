package frc.robot.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** The dashboard-driven sim opponent reports a center only when enabled (no hardware). */
class SimOpponentProviderTest {

  @BeforeAll
  static void once() {
    HAL.initialize(500, 0); // SmartDashboard goes through the NT/HAL
  }

  @Test
  void disabledByDefaultThenReportsConfiguredCenter() {
    SimOpponentProvider provider = new SimOpponentProvider();
    assertTrue(provider.dynamicObstacles().isEmpty(), "disabled → no obstacle");

    SmartDashboard.putBoolean("SimOpponentEnabled", true);
    SmartDashboard.putNumber("SimOpponentX", 5.0);
    SmartDashboard.putNumber("SimOpponentY", 3.0);
    SmartDashboard.putBoolean("SimOpponentOscillate", false);

    List<Translation2d> obstacles = provider.dynamicObstacles();
    assertEquals(1, obstacles.size(), "enabled → one obstacle");
    assertEquals(5.0, obstacles.get(0).getX(), 1e-9);
    assertEquals(3.0, obstacles.get(0).getY(), 1e-9);
  }
}
