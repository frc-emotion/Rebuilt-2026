package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;
import frc.robot.subsystems.indexer.IndexerIO.Stage;
import frc.robot.subsystems.indexer.IndexerIOSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Sim smoke tests: hood servos to a target; each indexer stage runs independently, both ways. */
class HoodIndexerSimTest {

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void hoodServosToMidPreset() {
    HoodIOSim io = new HoodIOSim();
    io.setTargetPosition(HoodConstants.kPresetMidRot);
    HoodIOInputs inputs = HoodIOInputs.kEmpty;
    for (int i = 0; i < 250; i++) {
      inputs = io.updateInputs();
    }
    assertEquals(HoodConstants.kPresetMidRot, inputs.positionRot(), HoodConstants.kToleranceRot);
  }

  @Test
  void indexerStagesRunIndependentlyAndReverse() {
    IndexerIOSim io = new IndexerIOSim();
    io.setVelocity(Stage.VERTICAL, 35.0);
    io.setVelocity(Stage.UPWARD, -50.0);
    IndexerIOInputs inputs = IndexerIOInputs.kEmpty;
    for (int i = 0; i < 250; i++) {
      inputs = io.updateInputs();
    }
    assertEquals(35.0, inputs.verticalVelocityRps(), 2.0);
    assertEquals(-50.0, inputs.upwardVelocityRps(), 2.0);
    assertTrue(Math.abs(inputs.horizontalVelocityRps()) < 0.5, "untouched stage must stay stopped");
  }
}
