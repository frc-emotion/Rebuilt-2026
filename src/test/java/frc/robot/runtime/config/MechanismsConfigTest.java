package frc.robot.runtime.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Locks the data layer: the deployed mechanisms.json parses, carries every tuned value VERBATIM,
 * and a malformed/missing file degrades to the documented safe fallback rather than a half-parsed
 * table.
 */
class MechanismsConfigTest {

  @BeforeAll
  static void setup() {
    HAL.initialize(500, 0);
  }

  @AfterAll
  static void teardown() {
    HAL.shutdown();
  }

  private static MechanismsConfig deployed() {
    Path file = Filesystem.getDeployDirectory().toPath().resolve("mechanisms.json");
    return MechanismsConfig.load(file);
  }

  @Test
  void deployedConfigLoadsAllEightMechanisms() {
    MechanismsConfig config = deployed();
    assertTrue(config.isValid(), "deployed mechanisms.json must be valid");
    assertEquals(8, config.size());
    for (String name :
        new String[] {
          "shooter",
          "turret",
          "hood",
          "indexerVertical",
          "indexerHorizontal",
          "indexerUpward",
          "intakePivot",
          "intakeRoller"
        }) {
      assertTrue(config.get(name).isPresent(), "missing mechanism " + name);
    }
  }

  @Test
  void shooterValuesAreVerbatim() {
    MechanismConfig s = deployed().get("shooter").orElseThrow();
    assertEquals(50, s.motorId());
    assertEquals(MechanismConfig.ControlType.VELOCITY, s.controlType());
    assertEquals(InvertedValue.CounterClockwise_Positive, s.inverted());
    assertEquals(NeutralModeValue.Coast, s.neutralMode());
    assertEquals(0.3, s.gains().kP());
    assertEquals(0.15, s.gains().kS());
    assertEquals(0.12, s.gains().kV());
    assertEquals(160.0, s.currentLimits().stator());
    assertEquals(120.0, s.currentLimits().supply());
    assertEquals(12.0, s.peakForwardVoltage());
    assertEquals(0.0, s.peakReverseVoltage());
    assertTrue(s.outputClamp().isPresent());
    assertEquals(0.0, s.outputClamp().get().min());
    assertEquals(400.0, s.outputClamp().get().max());
  }

  @Test
  void turretGearRatioReproducesTheJavaDoubleExactly() {
    MechanismConfig t = deployed().get("turret").orElseThrow();
    // Stored as "122/24" so it equals the legacy TurretConstants.kGearRatio double bit-for-bit.
    assertEquals(122.0 / 24.0, t.gearRatio());
    assertEquals(40.0, t.gains().kP());
    assertEquals(0.2, t.gains().kS());
    assertTrue(t.bootZero());
    assertTrue(t.softLimits().isPresent());
    assertEquals(0.39, t.softLimits().get().forward());
    assertEquals(-0.73, t.softLimits().get().reverse());
    assertEquals(FeedbackSensorSourceValue.RotorSensor, t.feedback().orElseThrow().source());
    assertFalse(t.encoder().orElseThrow().fused());
    assertTrue(t.sim().pinSoftLimits());
    assertTrue(t.sim().useKs());
    assertTrue(t.sim().useFeedforward());
  }

  @Test
  void hoodGearRatioAndClampAreVerbatim() {
    MechanismConfig h = deployed().get("hood").orElseThrow();
    assertEquals(155.0 / 12.0, h.gearRatio());
    assertEquals(100.0, h.gains().kP());
    assertEquals(50.0, h.gains().kI());
    assertEquals(0.0, h.outputClamp().orElseThrow().min());
    assertEquals(0.08, h.outputClamp().orElseThrow().max());
    assertFalse(h.sim().useKs());
    assertFalse(h.sim().useFeedforward());
  }

  @Test
  void intakePivotUsesFusedRemoteCancoder() {
    MechanismConfig p = deployed().get("intakePivot").orElseThrow();
    assertEquals(20, p.motorId());
    assertEquals(FeedbackSensorSourceValue.RemoteCANcoder, p.feedback().orElseThrow().source());
    assertEquals(22, p.feedback().orElseThrow().remoteSensorId());
    assertEquals(27.0, p.feedback().orElseThrow().rotorToSensorRatio());
    assertEquals(1.0, p.feedback().orElseThrow().sensorToMechanismRatio());
    assertFalse(p.bootZero());
    assertEquals(MechanismConfig.Sim.Plant.ARM, p.sim().plant());
    assertEquals(0.515, p.softLimits().orElseThrow().forward());
    assertEquals(0.14, p.softLimits().orElseThrow().reverse());
  }

  @Test
  void indexerStagesHaveDistinctIdsAndInversions() {
    MechanismsConfig c = deployed();
    assertEquals(32, c.get("indexerVertical").orElseThrow().motorId());
    assertEquals(31, c.get("indexerHorizontal").orElseThrow().motorId());
    assertEquals(33, c.get("indexerUpward").orElseThrow().motorId());
    // Horizontal is the odd one out (Clockwise); vertical + upward are CounterClockwise.
    assertEquals(
        InvertedValue.Clockwise_Positive, c.get("indexerHorizontal").orElseThrow().inverted());
    assertEquals(
        InvertedValue.CounterClockwise_Positive, c.get("indexerVertical").orElseThrow().inverted());
  }

  @Test
  void malformedJsonDegradesToSafeFallback() {
    MechanismsConfig bad = MechanismsConfig.parse("{ not valid json");
    assertFalse(bad.isValid());
    assertEquals(0, bad.size());
  }

  @Test
  void missingRequiredFieldDegradesToSafeFallback() {
    MechanismsConfig bad =
        MechanismsConfig.parse("{\"mechanisms\":[{\"name\":\"x\",\"controlType\":\"velocity\"}]}");
    assertFalse(bad.isValid(), "an entry missing motorId/bus/gains must not validate");
  }

  @Test
  void emptyMechanismsArrayIsInvalid() {
    assertFalse(MechanismsConfig.parse("{\"mechanisms\":[]}").isValid());
  }
}
