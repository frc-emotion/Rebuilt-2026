package frc.robot.superstructure;

/**
 * Everything the pure transition logic is allowed to know. Hood-at-setpoint is deliberately
 * absent: the legacy teleop feed gate ignored hood readiness (W13) and that is preserved.
 */
public record Conditions(
    boolean passSelected, // LB held (legacy isPassing supplier)
    boolean aimed, // |last vision tx| < 3° AND a fresh frame seen since enable (W10 fix)
    boolean atShooterSpeed, // |velocity − setpoint| < 1.67 RPS
    boolean shooterCommandedNonZero, // W12 fix: a 0-RPS setpoint can never open the feed gate
    boolean intakeDeployed, // intake >5° off stow (W18 semantics)
    boolean clearingElapsed) {} // CLEARING timer expired (timer lives in Superstructure)
