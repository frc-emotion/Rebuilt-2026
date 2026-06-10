package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

/**
 * Intake pivot + roller with a small NESTED state machine (the only one on the robot).
 *
 * <p>Justification: deploy sequencing is genuinely internal and temporal — rollers start only
 * within the loose deploy tolerance and then LATCH on (legacy W17), stow must stop the rollers
 * before traveling, and over-travel recovery (legacy W16) must keep running every loop regardless
 * of what the robot-level machine wants. These are mechanism-protection behaviors with their own
 * memory. This machine makes zero robot-level decisions; the Superstructure sees only
 * isOut()/isDeployed().
 */
@Logged
public class Intake extends SubsystemBase {

  public enum PivotState {
    STOWED,
    DEPLOYING,
    DEPLOYED_ROLLING,
    STOWING
  }

  private final IntakeIO io;
  private IntakeIOInputs inputs = IntakeIOInputs.kEmpty;

  @Logged(importance = Logged.Importance.CRITICAL)
  private PivotState pivotState = PivotState.STOWED;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean intakeIsOut = false;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean overtravelRecovery = false;

  @Logged(importance = Logged.Importance.DEBUG)
  private double pivotPositionRot = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double rollerVelocityRps = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double pivotCurrentAmps = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    inputs = io.updateInputs();
    pivotPositionRot = inputs.pivotPositionRot();
    rollerVelocityRps = inputs.rollerVelocityRps();
    pivotCurrentAmps = inputs.pivotSupplyCurrentAmps();

    // Legacy isOut threshold, verbatim: "out" means >5° away from stow (W18). The idle vertical
    // feed keys off this, so feeding starts as soon as game pieces can possibly enter.
    intakeIsOut =
        pivotPositionRot
            > IntakeConstants.kInAngle.in(Rotations) + IntakeConstants.kOutThresholdRot;

    pivotState =
        switch (pivotState) {
          case STOWED -> PivotState.STOWED;
          case DEPLOYING -> advanceDeploying();
          case DEPLOYED_ROLLING -> PivotState.DEPLOYED_ROLLING;
          case STOWING -> advanceStowing();
        };

    runOvertravelRecovery();
  }

  private PivotState advanceDeploying() {
    if (!withinOfTarget(
        IntakeConstants.kOutAngle.in(Rotations), IntakeConstants.kDeployTolerance.in(Rotations))) {
      return PivotState.DEPLOYING;
    }
    // Rollers latch ON here; a later pivot bounce outside tolerance must not stop them (W17).
    io.setRollerVelocity(IntakeConstants.kRollerVelocityRps);
    return PivotState.DEPLOYED_ROLLING;
  }

  private PivotState advanceStowing() {
    if (!withinOfTarget(
        IntakeConstants.kInAngle.in(Rotations), IntakeConstants.kTolerance.in(Rotations))) {
      return PivotState.STOWING;
    }
    return PivotState.STOWED;
  }

  // Over-travel recovery (legacy W16, verbatim semantics): if external forces push the intake
  // past the safe stow zone, re-command the stow position every loop so the PID fights back
  // before the mechanism mechanically jams. Only runs while stowing — never blocks a deploy.
  private void runOvertravelRecovery() {
    boolean tryingToStow = pivotState == PivotState.STOWED || pivotState == PivotState.STOWING;
    if (tryingToStow && pivotPositionRot < IntakeConstants.kOvertravelThresholdRot) {
      io.setPivotTarget(IntakeConstants.kInAngle.in(Rotations));
      overtravelRecovery = true;
    } else {
      overtravelRecovery = false;
    }
  }

  private boolean withinOfTarget(double targetRot, double toleranceRot) {
    return Math.abs(pivotPositionRot - targetRot) < toleranceRot;
  }

  /** Deploy: pivot out; rollers start automatically once within the deploy tolerance. */
  public void requestDeploy() {
    if (pivotState == PivotState.DEPLOYING || pivotState == PivotState.DEPLOYED_ROLLING) {
      return;
    }
    io.setPivotTarget(IntakeConstants.kOutAngle.in(Rotations));
    pivotState = PivotState.DEPLOYING;
  }

  /** Stow: rollers stop first (legacy end() semantics), then the pivot travels in. */
  public void requestStow() {
    if (pivotState == PivotState.STOWED || pivotState == PivotState.STOWING) {
      return;
    }
    io.stopRoller();
    io.setPivotTarget(IntakeConstants.kInAngle.in(Rotations));
    pivotState = PivotState.STOWING;
  }

  /** Legacy isOut(): pivot more than 5° off stow — NOT "fully deployed" (W18, load-bearing). */
  public boolean isOut() {
    return intakeIsOut;
  }

  public boolean isDeployRequested() {
    return pivotState == PivotState.DEPLOYING || pivotState == PivotState.DEPLOYED_ROLLING;
  }

  /** Fully deployed with rollers latched on (stricter than isOut — see W17/W18). */
  public boolean isDeployed() {
    return pivotState == PivotState.DEPLOYED_ROLLING;
  }

  /** Manual mode only: open-loop pivot jog; firmware soft limits remain the safety net. */
  public void setPivotVoltage(double volts) {
    pivotState = pivotPositionRot < IntakeConstants.kInAngle.in(Rotations) + 0.01
        ? PivotState.STOWED
        : PivotState.DEPLOYED_ROLLING;
    io.setPivotVoltage(volts);
  }

  /** Manual mode only: direct roller control. */
  public void setRollerVelocity(double rps) {
    io.setRollerVelocity(rps);
  }

  public void stop() {
    io.stop();
  }
}
