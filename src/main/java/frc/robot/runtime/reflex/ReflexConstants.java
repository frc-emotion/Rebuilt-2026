package frc.robot.runtime.reflex;

/**
 * Tuned numbers for the reflex library and the computed skills, ported VERBATIM from the legacy
 * IndexerConstants / IntakeConstants / Superstructure / TurretAiming. These are the real-time floor
 * (feed speeds, the clearing duration, the intake sequencing geometry, the aim deadband) and stay
 * as CODE forever — they never move to JSON and they never get rounded, converted, or "cleaned".
 *
 * <p>Each value's W# / legacy origin is noted where it is load-bearing.
 */
public final class ReflexConstants {
  private ReflexConstants() {}

  // ── Indexer feed speeds (legacy IndexerConstants) ──
  public static final double kVerticalSpeedRps = 35;
  public static final double kHorizontalSpeedRps = 35;
  public static final double kUpwardSpeedRps = 100;

  /** Rest vertical feed while intaking = 75% of full vertical (W18, legacy indexerDefault). */
  public static final double kIntakingVerticalSpeedRps = kVerticalSpeedRps * 0.75; // 26.25

  /** UNJAM reverses every stage at half speed (W19, legacy reverseIndexers). */
  public static final double kUnjamFraction = 0.5;

  /** CLEARING reverses every stage at FULL speed for this long (team decision, atomic). */
  public static final double kClearingSeconds = 2.0;

  // ── Shooter / hood scoring setpoints ──
  /** Passing shot is a fixed lob (legacy hardcoded, W14). */
  public static final double kPassingShooterRps = 95;

  /** Passing hood is a fixed lob angle (legacy hardcoded, W14). */
  public static final double kPassingHoodRot = 0.067;

  /** UNJAM spins the flywheel FORWARD flat-out (clamp ceiling = shooter kMaxRps, W19). */
  public static final double kUnjamShooterRps = 400;

  // ── Shooter readiness predicate (legacy ShooterConstants.kToleranceRps) ──
  public static final double kShooterToleranceRps = 1.67;

  // ── Turret aiming (legacy TurretAiming) ──
  public static final double kAimedDeadbandDeg = 3.0;
  public static final boolean kGyroFfEnabled = true; // W3

  // ── Intake nested machine (legacy IntakeConstants) ──
  public static final double kIntakeInAngleRot = 0.15;
  public static final double kIntakeOutAngleRot = 0.51;
  public static final double kIntakeReverseSoftLimitRot = 0.14;
  public static final double kIntakeOvertravelThresholdRot = 0.14; // W16 recovery trigger
  public static final double kIntakeForwardSoftLimitRot = 0.515;
  public static final double kIntakeToleranceRot = 5.0 / 360.0; // 5° (used for stow)
  public static final double kIntakeDeployToleranceRot = 15.0 / 360.0; // 15° (rollers latch, W17)
  public static final double kIntakeOutThresholdRot = 5.0 / 360.0; // >5° off stow = "out" (W18)
  public static final double kIntakeRollerVelocityRps = 40;

  // ── Staleness → safe (NEW reflex, not a ported tuned value) ──
  // Deliberately generous so it is inert in normal operation and during equivalence tests (which
  // feed a fresh perception frame every loop); it fires only on a genuinely dead pipeline.
  public static final double kPerceptionStaleSeconds = 1.0;
}
