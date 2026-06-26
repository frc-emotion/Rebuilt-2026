package frc.robot.runtime.reflex;

/**
 * The scoring-chain sequencer: the pure (phase, goal, conditions) → next-phase transition logic,
 * ported VERBATIM from the deleted superstructure {@code Transitions} (T# IDs preserved). It is the
 * enumerated reflex latch that gives the scoring axis its one-loop-per-transition timing — the
 * thing a purely stateless gate evaluation cannot reproduce (e.g. PASS_SPINNING_UP must precede
 * PASSING by a loop; SPINNING_UP must precede SHOOTING). The skill table declares each phase's
 * mechanism setpoints + feed; this decides which phase is active. Hardware-free and unit-tested,
 * exactly as the legacy transition logic was.
 *
 * <p>MANUAL is handled by the interpreter's mode layer before this is consulted (as the legacy
 * Superstructure did); a MANUAL input here stays MANUAL. The single ATOMIC transition — CLEARING
 * defers IDLE/PASS until clearingElapsed, while SHOOT re-request / UNJAM / manual exit instantly —
 * is preserved bit-for-bit.
 */
public final class ScoringSequencer {

  /** The scoring-chain phase (== the legacy RobotState). */
  public enum Phase {
    IDLE,
    INTAKING,
    SPINNING_UP,
    SHOOTING,
    CLEARING,
    PASS_AIMING,
    PASS_SPINNING_UP,
    PASSING,
    UNJAMMING,
    MANUAL
  }

  /** The caller's high-level intent (== the legacy Goal). */
  public enum Goal {
    IDLE,
    INTAKE,
    SHOOT,
    PASS,
    UNJAM
  }

  /** Everything the pure transition logic may know (== the legacy Conditions, W13: no hood). */
  public record Conditions(
      boolean passSelected,
      boolean aimed,
      boolean atShooterSpeed,
      boolean shooterCommandedNonZero,
      boolean intakeDeployed,
      boolean clearingElapsed) {}

  private ScoringSequencer() {}

  public static Phase next(Phase state, Goal goal, Conditions c) {
    return switch (state) {
      case IDLE, INTAKING -> fromRest(goal, c); // T0–T4
      case SPINNING_UP -> fromSpinningUp(goal, c);
      case SHOOTING -> fromShooting(goal, c);
      case CLEARING -> fromClearing(goal, c);
      case PASS_AIMING -> fromPassAiming(goal, c);
      case PASS_SPINNING_UP -> fromPassSpinningUp(goal, c);
      case PASSING -> fromPassing(goal, c);
      case UNJAMMING -> fromUnjamming(goal, c);
      case MANUAL -> Phase.MANUAL; // T20/T21 live in the interpreter's mode layer
    };
  }

  /** The no-request resting state: INTAKING when the intake toggle is on, else IDLE (T0). */
  private static Phase rest(Conditions c) {
    return c.intakeDeployed() ? Phase.INTAKING : Phase.IDLE;
  }

  private static Phase fromRest(Goal goal, Conditions c) {
    return switch (goal) {
      case IDLE, INTAKE -> rest(c);
      case SHOOT -> c.passSelected() ? Phase.PASS_SPINNING_UP : Phase.SPINNING_UP; // T1/T2
      case PASS -> Phase.PASS_AIMING; // T3
      case UNJAM -> Phase.UNJAMMING; // T4
    };
  }

  private static Phase fromSpinningUp(Goal goal, Conditions c) {
    if (goal != Goal.SHOOT) {
      return fromRest(goal, c); // T7 — no balls committed yet, no clearing
    }
    if (c.passSelected()) {
      return Phase.PASS_SPINNING_UP; // T6
    }
    if (c.aimed() && c.atShooterSpeed() && c.shooterCommandedNonZero()) {
      return Phase.SHOOTING; // T5 — the full gate, including the W12 zero-setpoint guard
    }
    return Phase.SPINNING_UP;
  }

  private static Phase fromShooting(Goal goal, Conditions c) {
    if (goal == Goal.UNJAM) {
      return Phase.UNJAMMING; // T10u — recovery always preempts
    }
    if (goal != Goal.SHOOT) {
      return Phase.CLEARING; // T10 [ATOMIC] — balls are committed, back them out
    }
    if (c.passSelected()) {
      return Phase.PASS_SPINNING_UP; // T9
    }
    if (!(c.aimed() && c.atShooterSpeed())) {
      return Phase.SPINNING_UP; // T8 — anti-dribble re-close (W13) / aim-loss fix (W12)
    }
    return Phase.SHOOTING;
  }

  private static Phase fromClearing(Goal goal, Conditions c) {
    if (goal == Goal.UNJAM) {
      return Phase.UNJAMMING; // T24 — preempts the atomic clear
    }
    if (goal == Goal.SHOOT) {
      return c.passSelected() ? Phase.PASS_SPINNING_UP : Phase.SPINNING_UP; // T23
    }
    if (c.clearingElapsed()) {
      return goal == Goal.PASS ? Phase.PASS_AIMING : rest(c); // T22
    }
    return Phase.CLEARING; // atomic: IDLE/PASS wait for the clear to finish
  }

  private static Phase fromPassAiming(Goal goal, Conditions c) {
    return switch (goal) {
      case IDLE, INTAKE -> rest(c); // T12
      case SHOOT -> c.passSelected() ? Phase.PASS_SPINNING_UP : Phase.SPINNING_UP; // T11
      case PASS -> Phase.PASS_AIMING;
      case UNJAM -> Phase.UNJAMMING; // T13
    };
  }

  private static Phase fromPassSpinningUp(Goal goal, Conditions c) {
    if (goal != Goal.SHOOT) {
      return fromRest(goal, c); // T16
    }
    if (!c.passSelected()) {
      return Phase.SPINNING_UP; // T15 — LB released mid-spin, fall back to the hub chain
    }
    if (c.atShooterSpeed()) {
      return Phase.PASSING; // T14 — passing gates on speed only (legacy quirk, preserved)
    }
    return Phase.PASS_SPINNING_UP;
  }

  private static Phase fromPassing(Goal goal, Conditions c) {
    if (goal == Goal.UNJAM) {
      return Phase.UNJAMMING; // T18u
    }
    if (goal != Goal.SHOOT) {
      return Phase.CLEARING; // T18 [ATOMIC]
    }
    if (!c.passSelected()) {
      return Phase.SPINNING_UP; // T15 mirror — LB released while firing
    }
    if (!c.atShooterSpeed()) {
      return Phase.PASS_SPINNING_UP; // T17
    }
    return Phase.PASSING;
  }

  private static Phase fromUnjamming(Goal goal, Conditions c) {
    if (goal == Goal.UNJAM) {
      return Phase.UNJAMMING;
    }
    return fromRest(goal, c); // T19
  }
}
