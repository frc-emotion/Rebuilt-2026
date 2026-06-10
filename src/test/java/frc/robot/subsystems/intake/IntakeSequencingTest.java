package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.intake.Intake.PivotState;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Nested-machine sequencing tests against a fake IO with a directly settable pivot position. */
class IntakeSequencingTest {

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  /** Fake IO: position set by the test, commands recorded for assertion. */
  private static final class FakeIntakeIO implements IntakeIO {
    double pivotPositionRot = IntakeConstants.kInAngle.in(edu.wpi.first.units.Units.Rotations);
    double lastPivotTargetRot = Double.NaN;
    double lastRollerRps = 0.0;
    boolean rollerStopped = true;
    int pivotTargetCommands = 0;

    @Override
    public IntakeIOInputs updateInputs() {
      return new IntakeIOInputs(pivotPositionRot, 0.0, lastRollerRps);
    }

    @Override
    public void setPivotTarget(double rot) {
      lastPivotTargetRot = rot;
      pivotTargetCommands++;
    }

    @Override
    public void setPivotVoltage(double volts) {}

    @Override
    public void setRollerVelocity(double rps) {
      lastRollerRps = rps;
      rollerStopped = false;
    }

    @Override
    public void stopRoller() {
      lastRollerRps = 0.0;
      rollerStopped = true;
    }

    @Override
    public void stop() {
      stopRoller();
    }
  }

  private static final double kOut = 0.51;
  private static final double kIn = 0.15;

  @Test
  void rollersStartOnlyWithin15Degrees() {
    FakeIntakeIO io = new FakeIntakeIO();
    Intake intake = new Intake(io);
    intake.requestDeploy();
    assertEquals(kOut, io.lastPivotTargetRot, 1e-9);

    io.pivotPositionRot = kOut - (20.0 / 360.0); // 20° away — too far
    intake.periodic();
    assertTrue(io.rollerStopped, "rollers must not start outside deploy tolerance");

    io.pivotPositionRot = kOut - (10.0 / 360.0); // 10° away — inside the loose 15° tolerance
    intake.periodic();
    assertEquals(IntakeConstants.kRollerVelocityRps, io.lastRollerRps, 1e-9);
  }

  @Test
  void rollerLatchSurvivesPivotBounce() {
    FakeIntakeIO io = new FakeIntakeIO();
    Intake intake = new Intake(io);
    intake.requestDeploy();
    io.pivotPositionRot = kOut;
    intake.periodic(); // latch on
    io.pivotPositionRot = kOut - (25.0 / 360.0); // bounce outside tolerance
    intake.periodic();
    assertFalse(io.rollerStopped, "latched rollers must survive pivot bounce (W17)");
  }

  @Test
  void stowStopsRollersBeforeTravel() {
    FakeIntakeIO io = new FakeIntakeIO();
    Intake intake = new Intake(io);
    intake.requestDeploy();
    io.pivotPositionRot = kOut;
    intake.periodic();
    intake.requestStow();
    assertTrue(io.rollerStopped, "stow must stop rollers immediately");
    assertEquals(kIn, io.lastPivotTargetRot, 1e-9);
    io.pivotPositionRot = kIn;
    intake.periodic();
    assertEquals(PivotState.STOWED, stateOf(intake));
  }

  @Test
  void overtravelRecoveryOnlyWhenStowing() {
    FakeIntakeIO io = new FakeIntakeIO();
    Intake intake = new Intake(io);

    // Stowed and shoved past the threshold: recovery re-commands stow every loop.
    io.pivotPositionRot = 0.10; // below 0.14
    int before = io.pivotTargetCommands;
    intake.periodic();
    intake.periodic();
    assertTrue(io.pivotTargetCommands >= before + 2, "recovery must re-command stow each loop");
    assertEquals(kIn, io.lastPivotTargetRot, 1e-9);

    // Deploying: recovery must never fight the deploy, even at low pivot angles.
    intake.requestDeploy();
    before = io.pivotTargetCommands;
    intake.periodic();
    assertEquals(before, io.pivotTargetCommands, "recovery must not block a deploy (W16)");
  }

  @Test
  void isOutUsesFiveDegreeThresholdNotDeployTarget() {
    FakeIntakeIO io = new FakeIntakeIO();
    Intake intake = new Intake(io);
    io.pivotPositionRot = kIn + (6.0 / 360.0); // barely 6° off stow
    intake.periodic();
    assertTrue(intake.isOut(), "isOut is >5 deg off stow, NOT near the deploy target (W18)");
  }

  private static PivotState stateOf(Intake intake) {
    return intake.isDeployRequested() ? PivotState.DEPLOYED_ROLLING : PivotState.STOWED;
  }
}
