package frc.robot.superstructure;

/**
 * What the caller wants. Operator buttons and auto routines request goals, never states. INTAKE is
 * the one goal that composes with the others — it is a toggle on an orthogonal axis
 * (Superstructure.toggleIntake) and surfaces in the scoring machine as Conditions.intakeDeployed.
 */
public enum Goal {
  IDLE,
  INTAKE,
  SHOOT,
  PASS,
  UNJAM
}
