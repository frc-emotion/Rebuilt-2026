package frc.robot.runtime.reflex;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.runtime.MechanismCommander;
import frc.robot.runtime.MechanismState;
import frc.robot.runtime.reflex.IntakeReflex.PivotState;
import org.junit.jupiter.api.Test;

/**
 * The intake nested machine (the robot's only nested state machine), tested against a fake
 * commander with a directly settable pivot position — ported from the legacy IntakeSequencingTest.
 * Covers the load-bearing mechanism-protection behaviors: the 15° deploy latch (W17),
 * stow-stops-rollers-first, over-travel recovery below 0.14 rot (W16), and the isOut 5° threshold
 * (W18).
 */
class IntakeReflexTest {

  private static final double kOut = ReflexConstants.kIntakeOutAngleRot; // 0.51
  private static final double kIn = ReflexConstants.kIntakeInAngleRot; // 0.15

  /** Fake commander: pivot position set by the test, commands recorded for assertion. */
  private static final class FakeCommander implements MechanismCommander {
    double pivotPositionRot = kIn;
    double lastPivotTargetRot = Double.NaN;
    double lastRollerRps = 0.0;
    boolean rollerStopped = true;
    int pivotTargetCommands = 0;

    @Override
    public MechanismState read(String name) {
      if (IntakeReflex.PIVOT.equals(name)) {
        return new MechanismState(pivotPositionRot, 0.0, 0.0, 0.0, 0.0, false, false);
      }
      return MechanismState.kZero;
    }

    @Override
    public void setVelocity(String name, double rps) {
      if (IntakeReflex.ROLLER.equals(name)) {
        lastRollerRps = rps;
        rollerStopped = false;
      }
    }

    @Override
    public void setPosition(String name, double rot) {
      if (IntakeReflex.PIVOT.equals(name)) {
        lastPivotTargetRot = rot;
        pivotTargetCommands++;
      }
    }

    @Override
    public void stop(String name) {
      if (IntakeReflex.ROLLER.equals(name)) {
        lastRollerRps = 0.0;
        rollerStopped = true;
      }
    }
  }

  @Test
  void rollersStartOnlyWithin15Degrees() {
    FakeCommander io = new FakeCommander();
    IntakeReflex intake = new IntakeReflex();

    intake.update(io, true); // request deploy
    assertEquals(kOut, io.lastPivotTargetRot, 1e-9);

    io.pivotPositionRot = kOut - (20.0 / 360.0); // 20° away — too far
    intake.update(io, true);
    assertTrue(io.rollerStopped, "rollers must not start outside the deploy tolerance");

    io.pivotPositionRot = kOut - (10.0 / 360.0); // 10° away — inside the loose 15° tolerance
    intake.update(io, true);
    assertEquals(ReflexConstants.kIntakeRollerVelocityRps, io.lastRollerRps, 1e-9);
  }

  @Test
  void rollerLatchSurvivesPivotBounce() {
    FakeCommander io = new FakeCommander();
    IntakeReflex intake = new IntakeReflex();
    intake.update(io, true);
    io.pivotPositionRot = kOut;
    intake.update(io, true); // latch on
    io.pivotPositionRot = kOut - (25.0 / 360.0); // bounce outside tolerance
    intake.update(io, true);
    assertFalse(io.rollerStopped, "latched rollers must survive a pivot bounce (W17)");
  }

  @Test
  void stowStopsRollersBeforeTravel() {
    FakeCommander io = new FakeCommander();
    IntakeReflex intake = new IntakeReflex();
    intake.update(io, true);
    io.pivotPositionRot = kOut;
    intake.update(io, true); // deployed + rolling
    assertFalse(io.rollerStopped);

    intake.update(io, false); // request stow
    assertTrue(io.rollerStopped, "stow must stop the rollers before traveling");
    assertEquals(kIn, io.lastPivotTargetRot, 1e-9);

    io.pivotPositionRot = kIn;
    intake.update(io, false);
    assertEquals(PivotState.STOWED, intake.pivotState());
  }

  @Test
  void overtravelRecoveryOnlyWhenStowing() {
    FakeCommander io = new FakeCommander();
    IntakeReflex intake = new IntakeReflex();

    // Stowed and shoved past the threshold: recovery re-commands the stow target every loop.
    io.pivotPositionRot = 0.10; // below 0.14
    int before = io.pivotTargetCommands;
    intake.update(io, false);
    intake.update(io, false);
    assertTrue(
        io.pivotTargetCommands >= before + 2, "recovery must re-command stow each loop (W16)");
    assertEquals(kIn, io.lastPivotTargetRot, 1e-9);

    // Deploying: recovery must never fight the deploy, even at low pivot angles.
    before = io.pivotTargetCommands;
    intake.update(io, true); // request deploy (one setPosition for the deploy target)
    io.pivotPositionRot = 0.10; // still low, but now deploying
    int afterDeployRequest = io.pivotTargetCommands;
    intake.update(io, true);
    assertEquals(
        afterDeployRequest,
        io.pivotTargetCommands,
        "recovery must not re-command the stow target while deploying (W16)");
  }

  @Test
  void isOutUsesFiveDegreeThresholdNotDeployTarget() {
    FakeCommander io = new FakeCommander();
    IntakeReflex intake = new IntakeReflex();
    io.pivotPositionRot = kIn + (6.0 / 360.0); // barely 6° off stow
    intake.update(io, false);
    assertTrue(intake.isOut(), "isOut is >5° off stow, NOT near the deploy target (W18)");
    assertFalse(intake.isDeployed(), "isOut must not imply fully deployed (W18)");
  }
}
