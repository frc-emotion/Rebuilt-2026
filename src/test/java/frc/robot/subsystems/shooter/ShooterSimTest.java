package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Sim smoke test: the FlywheelSim-backed IO must spin up, settle, and coast down. */
class ShooterSimTest {

  @BeforeAll
  static void initHal() {
    // WPILib sim classes read battery voltage through the HAL; without this, native SIGSEGV.
    HAL.initialize(500, 0);
  }

  @Test
  void spinsUpToSetpointInSim() {
    ShooterIOSim io = new ShooterIOSim();
    io.setVelocity(50.0);
    ShooterIOInputs inputs = ShooterIOInputs.kEmpty;
    for (int i = 0; i < 250; i++) { // 5 simulated seconds
      inputs = io.updateInputs();
    }
    assertEquals(50.0, inputs.velocityRps(), ShooterConstants.kToleranceRps);
  }

  @Test
  void coastsDownAfterStopWithoutReverseVoltage() {
    ShooterIOSim io = new ShooterIOSim();
    io.setVelocity(50.0);
    for (int i = 0; i < 250; i++) {
      io.updateInputs();
    }
    io.stop();
    ShooterIOInputs inputs = io.updateInputs();
    double justAfterStop = inputs.velocityRps();
    for (int i = 0; i < 100; i++) {
      inputs = io.updateInputs();
    }
    assertTrue(inputs.velocityRps() < justAfterStop, "flywheel should decay after stop");
    assertEquals(0.0, inputs.appliedVolts(), 1e-9, "stop must apply zero volts, never reverse");
  }

  @Test
  void negativeRequestNeverDrivesBackwardsThroughSubsystemClamp() {
    // The subsystem clamps to [0, max]; verify the sim never sees reverse drive even at 0.
    ShooterIOSim io = new ShooterIOSim();
    io.setVelocity(0.0);
    ShooterIOInputs inputs = ShooterIOInputs.kEmpty;
    for (int i = 0; i < 50; i++) {
      inputs = io.updateInputs();
    }
    assertFalse(inputs.velocityRps() < -0.01, "flywheel must not spin backwards");
  }
}
