package frc.robot.runtime.reflex;

import edu.wpi.first.epilogue.Logged;
import frc.robot.runtime.MechanismCommander;

/**
 * The intake deploy/stow nested state machine, ported VERBATIM from the legacy Intake subsystem —
 * the robot's only nested machine and a pure mechanism-protection reflex. Deploy sequencing is
 * temporal: rollers latch ON within the loose 15° tolerance and stay on through a later pivot
 * bounce (W17); stow stops the rollers BEFORE traveling; over-travel recovery (W16) re-commands the
 * stow target every loop below 0.14 rot so the PID fights external forces before a mechanical jam.
 * It makes zero robot-level decisions — the interpreter reads only {@link #isOut()} / {@link
 * #isDeployed()}.
 *
 * <p>Within-loop order matches the legacy scheduler exactly: advance the machine + run over-travel
 * recovery using THIS loop's pivot reading, THEN apply the deploy/stow request (so a request issued
 * this loop drives next loop's advance, as it did when Intake.periodic ran before the
 * Superstructure).
 */
@Logged
public class IntakeReflex {
  public static final String PIVOT = "intakePivot";
  public static final String ROLLER = "intakeRoller";

  public enum PivotState {
    STOWED,
    DEPLOYING,
    DEPLOYED_ROLLING,
    STOWING
  }

  @Logged(importance = Logged.Importance.CRITICAL)
  private PivotState pivotState = PivotState.STOWED;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean intakeIsOut = false;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean overtravelRecovery = false;

  private double pivotPositionRot = 0.0;

  /** Re-seed on enable: the pivot machine simply re-reads hardware, no latch to clear. */
  public void reset() {
    // The pivot state is derived from the absolute CANcoder each loop; nothing to reset here beyond
    // letting update() re-evaluate. Kept for lifecycle symmetry with the other reflexes.
  }

  /** One loop of the intake axis. {@code deployRequested} is the operator A toggle state. */
  public void update(MechanismCommander m, boolean deployRequested) {
    pivotPositionRot = m.read(PIVOT).positionRot();

    // Legacy isOut threshold, verbatim: "out" means >5° away from stow (W18). The idle vertical
    // feed keys off this, so feeding starts as soon as game pieces can possibly enter.
    intakeIsOut =
        pivotPositionRot
            > ReflexConstants.kIntakeInAngleRot + ReflexConstants.kIntakeOutThresholdRot;

    pivotState =
        switch (pivotState) {
          case STOWED -> PivotState.STOWED;
          case DEPLOYING -> advanceDeploying(m);
          case DEPLOYED_ROLLING -> PivotState.DEPLOYED_ROLLING;
          case STOWING -> advanceStowing();
        };

    runOvertravelRecovery(m);

    if (deployRequested) {
      requestDeploy(m);
    } else {
      requestStow(m);
    }
  }

  private PivotState advanceDeploying(MechanismCommander m) {
    if (!withinOfTarget(
        ReflexConstants.kIntakeOutAngleRot, ReflexConstants.kIntakeDeployToleranceRot)) {
      return PivotState.DEPLOYING;
    }
    // Rollers latch ON here; a later pivot bounce outside tolerance must not stop them (W17).
    m.setVelocity(ROLLER, ReflexConstants.kIntakeRollerVelocityRps);
    return PivotState.DEPLOYED_ROLLING;
  }

  private PivotState advanceStowing() {
    if (!withinOfTarget(ReflexConstants.kIntakeInAngleRot, ReflexConstants.kIntakeToleranceRot)) {
      return PivotState.STOWING;
    }
    return PivotState.STOWED;
  }

  // Over-travel recovery (legacy W16, verbatim): if external forces push the intake past the safe
  // stow zone, re-command the stow position every loop so the PID fights back before the mechanism
  // mechanically jams. Only runs while stowing — never blocks a deploy.
  private void runOvertravelRecovery(MechanismCommander m) {
    boolean tryingToStow = pivotState == PivotState.STOWED || pivotState == PivotState.STOWING;
    if (tryingToStow && pivotPositionRot < ReflexConstants.kIntakeOvertravelThresholdRot) {
      m.setPosition(PIVOT, ReflexConstants.kIntakeInAngleRot);
      overtravelRecovery = true;
    } else {
      overtravelRecovery = false;
    }
  }

  private boolean withinOfTarget(double targetRot, double toleranceRot) {
    return Math.abs(pivotPositionRot - targetRot) < toleranceRot;
  }

  /** Deploy: pivot out; rollers start automatically once within the deploy tolerance. */
  private void requestDeploy(MechanismCommander m) {
    if (pivotState == PivotState.DEPLOYING || pivotState == PivotState.DEPLOYED_ROLLING) {
      return;
    }
    m.setPosition(PIVOT, ReflexConstants.kIntakeOutAngleRot);
    pivotState = PivotState.DEPLOYING;
  }

  /** Stow: rollers stop first (legacy end() semantics), then the pivot travels in. */
  private void requestStow(MechanismCommander m) {
    if (pivotState == PivotState.STOWED || pivotState == PivotState.STOWING) {
      return;
    }
    m.stop(ROLLER);
    m.setPosition(PIVOT, ReflexConstants.kIntakeInAngleRot);
    pivotState = PivotState.STOWING;
  }

  /** Legacy isOut(): pivot more than 5° off stow — NOT "fully deployed" (W18, load-bearing). */
  public boolean isOut() {
    return intakeIsOut;
  }

  /** Fully deployed with rollers latched on (stricter than isOut — see W17/W18). */
  public boolean isDeployed() {
    return pivotState == PivotState.DEPLOYED_ROLLING;
  }

  public PivotState pivotState() {
    return pivotState;
  }
}
