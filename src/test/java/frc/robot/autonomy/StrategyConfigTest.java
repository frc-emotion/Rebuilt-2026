package frc.robot.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.autonomy.ShiftSchedule.Mode;
import frc.robot.autonomy.StrategyConfig.Plan;
import java.util.Optional;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Tests the strategy.json loader: the real deployed file validates, and bad input safe-falls. */
class StrategyConfigTest {

  @BeforeAll
  static void once() {
    HAL.initialize(500, 0); // DriverStation.reportError on the invalid paths routes through the HAL
  }

  @Test
  void deployedStrategyLoadsAndResolves() {
    StrategyConfig cfg =
        StrategyConfig.load(Filesystem.getDeployDirectory().toPath().resolve("strategy.json"));
    assertTrue(cfg.isValid(), "deploy/strategy.json must be valid");
    assertEquals(4.5, cfg.returnLeadSeconds(), 1e-9);
    assertTrue(cfg.pose("shoot_main").isPresent(), "shoot_main location must resolve");

    Plan active = cfg.plan(Mode.OUR_HUB_ACTIVE).orElseThrow();
    assertEquals(Optional.of("shoot_main"), active.shootFrom());
    assertEquals(2, cfg.route(active.collectRoute()).size());

    Plan inactive = cfg.plan(Mode.OUR_HUB_INACTIVE).orElseThrow();
    assertTrue(inactive.stageAt().isPresent(), "harvest plan must name a stage location");
    assertFalse(cfg.route(inactive.harvestRoute()).isEmpty(), "harvest plan must have a route");
  }

  @Test
  void emptyOrMalformedFallsBackSafely() {
    assertFalse(StrategyConfig.parse("{}").isValid(), "missing locations/plans -> invalid");
    assertFalse(StrategyConfig.parse("not json").isValid(), "garbage -> invalid");
    assertEquals(
        4.5,
        StrategyConfig.safeFallback().returnLeadSeconds(),
        1e-9,
        "safe fallback keeps a sane default lead");
  }

  @Test
  void unknownLocationReferenceIsRejected() {
    String bad =
        "{\"locations\":{\"a\":{\"x\":1,\"y\":1}},"
            + "\"plans\":{\"OUR_HUB_ACTIVE\":{\"shootFrom\":\"missing\"}}}";
    assertFalse(
        StrategyConfig.parse(bad).isValid(), "a plan referencing an unknown location -> invalid");
  }
}
