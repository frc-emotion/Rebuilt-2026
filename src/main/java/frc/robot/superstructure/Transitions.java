package frc.robot.superstructure;

/**
 * Pure transition logic: (state, goal, conditions) → next state. No hardware, no subsystems — this
 * is the JUnit-tested heart of the robot. Transition IDs (T#) match REFACTOR_DESIGN.md §2 and
 * docs/superstructure.md.
 *
 * <p>MANUAL entry/exit (T20/T21) is a mode toggle handled by the Superstructure BEFORE this
 * function is consulted; a MANUAL input here stays MANUAL.
 *
 * <p>The single ATOMIC transition: CLEARING defers IDLE/PASS goal changes until clearingElapsed.
 * SHOOT re-request, UNJAM, and the manual toggle all exit it immediately.
 */
public final class Transitions {
  private Transitions() {}

  public static RobotState next(RobotState state, Goal goal, Conditions c) {
    return switch (state) {
      case IDLE, INTAKING -> fromRest(goal, c); // T0–T4
      case SPINNING_UP -> fromSpinningUp(goal, c);
      case SHOOTING -> fromShooting(goal, c);
      case CLEARING -> fromClearing(goal, c);
      case PASS_AIMING -> fromPassAiming(goal, c);
      case PASS_SPINNING_UP -> fromPassSpinningUp(goal, c);
      case PASSING -> fromPassing(goal, c);
      case UNJAMMING -> fromUnjamming(goal, c);
      case MANUAL -> RobotState.MANUAL; // T20/T21 live in the Superstructure mode layer
    };
  }

  /** The no-request resting state: INTAKING when the intake toggle is on, else IDLE (T0). */
  private static RobotState rest(Conditions c) {
    return c.intakeDeployed() ? RobotState.INTAKING : RobotState.IDLE;
  }

  private static RobotState fromRest(Goal goal, Conditions c) {
    return switch (goal) {
      case IDLE, INTAKE -> rest(c);
      case SHOOT ->
          c.passSelected() ? RobotState.PASS_SPINNING_UP : RobotState.SPINNING_UP; // T1/T2
      case PASS -> RobotState.PASS_AIMING; // T3
      case UNJAM -> RobotState.UNJAMMING; // T4
    };
  }

  private static RobotState fromSpinningUp(Goal goal, Conditions c) {
    if (goal != Goal.SHOOT) {
      return fromRest(goal, c); // T7 — no balls committed yet, no clearing
    }
    if (c.passSelected()) {
      return RobotState.PASS_SPINNING_UP; // T6
    }
    if (c.aimed() && c.atShooterSpeed() && c.shooterCommandedNonZero()) {
      return RobotState.SHOOTING; // T5 — the full gate, including the W12 zero-setpoint guard
    }
    return RobotState.SPINNING_UP;
  }

  private static RobotState fromShooting(Goal goal, Conditions c) {
    if (goal == Goal.UNJAM) {
      return RobotState.UNJAMMING; // T10u — recovery always preempts
    }
    if (goal != Goal.SHOOT) {
      return RobotState.CLEARING; // T10 [ATOMIC] — balls are committed, back them out
    }
    if (c.passSelected()) {
      return RobotState.PASS_SPINNING_UP; // T9
    }
    if (!(c.aimed() && c.atShooterSpeed())) {
      return RobotState.SPINNING_UP; // T8 — anti-dribble re-close (W13) / aim-loss fix (W12)
    }
    return RobotState.SHOOTING;
  }

  private static RobotState fromClearing(Goal goal, Conditions c) {
    if (goal == Goal.UNJAM) {
      return RobotState.UNJAMMING; // T24 — preempts the atomic clear
    }
    if (goal == Goal.SHOOT) {
      return c.passSelected() ? RobotState.PASS_SPINNING_UP : RobotState.SPINNING_UP; // T23
    }
    if (c.clearingElapsed()) {
      return goal == Goal.PASS ? RobotState.PASS_AIMING : rest(c); // T22
    }
    return RobotState.CLEARING; // atomic: IDLE/PASS wait for the clear to finish
  }

  private static RobotState fromPassAiming(Goal goal, Conditions c) {
    return switch (goal) {
      case IDLE, INTAKE -> rest(c); // T12
      case SHOOT -> c.passSelected() ? RobotState.PASS_SPINNING_UP : RobotState.SPINNING_UP; // T11
      case PASS -> RobotState.PASS_AIMING;
      case UNJAM -> RobotState.UNJAMMING; // T13
    };
  }

  private static RobotState fromPassSpinningUp(Goal goal, Conditions c) {
    if (goal != Goal.SHOOT) {
      return fromRest(goal, c); // T16
    }
    if (!c.passSelected()) {
      return RobotState.SPINNING_UP; // T15 — LB released mid-spin, fall back to the hub chain
    }
    if (c.atShooterSpeed()) {
      return RobotState.PASSING; // T14 — passing gates on speed only (legacy quirk, preserved)
    }
    return RobotState.PASS_SPINNING_UP;
  }

  private static RobotState fromPassing(Goal goal, Conditions c) {
    if (goal == Goal.UNJAM) {
      return RobotState.UNJAMMING; // T18u
    }
    if (goal != Goal.SHOOT) {
      return RobotState.CLEARING; // T18 [ATOMIC]
    }
    if (!c.passSelected()) {
      return RobotState.SPINNING_UP; // T15 mirror — LB released while firing
    }
    if (!c.atShooterSpeed()) {
      return RobotState.PASS_SPINNING_UP; // T17
    }
    return RobotState.PASSING;
  }

  private static RobotState fromUnjamming(Goal goal, Conditions c) {
    if (goal == Goal.UNJAM) {
      return RobotState.UNJAMMING;
    }
    return fromRest(goal, c); // T19
  }
}
