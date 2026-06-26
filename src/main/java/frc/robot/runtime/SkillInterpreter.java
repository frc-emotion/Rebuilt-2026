package frc.robot.runtime;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.runtime.Setpoints.SetpointContext;
import frc.robot.runtime.config.MechanismConfig;
import frc.robot.runtime.config.MechanismsConfig;
import frc.robot.runtime.config.SkillSpec;
import frc.robot.runtime.config.SkillTable;
import frc.robot.runtime.perception.PerceptionProvider;
import frc.robot.runtime.reflex.HoodHold;
import frc.robot.runtime.reflex.IndexerFeed;
import frc.robot.runtime.reflex.IntakeReflex;
import frc.robot.runtime.reflex.ReflexConstants;
import frc.robot.runtime.reflex.ScoringSequencer;
import frc.robot.runtime.reflex.ScoringSequencer.Conditions;
import frc.robot.runtime.reflex.ScoringSequencer.Goal;
import frc.robot.runtime.reflex.ScoringSequencer.Phase;
import frc.robot.runtime.reflex.StalenessReflex;
import java.util.OptionalDouble;

/**
 * The stateless skill interpreter — the heart of the skill-server architecture, replacing the
 * deleted Goal/RobotState/Transitions/Superstructure. Each loop it maps the requested high-level
 * skill to a goal + conditions, advances the {@link ScoringSequencer} one transition, executes that
 * phase's mechanism setpoints (resolving the skill table's named SOURCES through {@link Setpoints}
 * / {@link TurretAiming}, picking the indexer {@link IndexerFeed} pattern from the phase), applies
 * the reflexes (turret tracking, hood capture-once, staleness→safe, the intake nested machine),
 * writes setpoints through {@link Mechanisms}, and publishes status. It runs the intake axis every
 * loop (every mode, including manual); drive is handled separately by the {@code Drive} adapter +
 * PathPlanner.
 *
 * <p>The enumerated cross-loop reflex latches are: the scoring phase + CLEARING timer (this class +
 * {@link ScoringSequencer}), the intake nested machine ({@link IntakeReflex}), the turret aiming
 * accumulator ({@link TurretAiming}), the hood capture-once ({@link HoodHold}), the staleness timer
 * ({@link StalenessReflex}), and a one-bit manual-mode edge detector. Phase behavior is the legacy
 * applyStateBehavior, verbatim.
 */
public class SkillInterpreter {
  private static final String TURRET = "turret";
  private static final String HOOD = "hood";
  private static final String SHOOTER = "shooter";

  // Fallback turret soft limits if the config omits them (legacy TurretConstants values).
  private static final double kDefaultReverseLimitRot = -0.73;
  private static final double kDefaultForwardLimitRot = 0.39;

  /** Everything the interpreter needs from the request side each loop (operator or coprocessor). */
  public record InterpreterInputs(
      String scoringSkill,
      boolean manualMode,
      boolean manualFeedHeld,
      boolean intakeDeployRequested) {}

  public enum AxisStatus {
    OK,
    RUNNING,
    FAIL
  }

  private final Mechanisms mechanisms;
  private final Setpoints setpoints;
  private final PerceptionProvider perception;
  private final ChassisStateProvider chassis;
  private final SkillTable skills;

  private final double turretReverseLimitRot;
  private final double turretForwardLimitRot;

  // Named setpoint SOURCES resolved from the skill table (so the JSON owns the references).
  private final String hubShooterSrc;
  private final String hubHoodSrc;
  private final String passShooterSrc;
  private final String passHoodSrc;
  private final String unjamShooterSrc;

  // Reflex latches (the only cross-loop state).
  private final TurretAiming aiming = new TurretAiming();
  private final IntakeReflex intake = new IntakeReflex();
  private final HoodHold hoodHold = new HoodHold();
  private final StalenessReflex staleness = new StalenessReflex();
  private final Timer clearingTimer = new Timer();
  private Phase phase = Phase.IDLE;
  private boolean prevManualMode = false;

  private AxisStatus scoringStatus = AxisStatus.OK;
  private AxisStatus intakeStatus = AxisStatus.OK;
  private String activeScoringSkill = "idle";

  public SkillInterpreter(
      Mechanisms mechanisms,
      Setpoints setpoints,
      PerceptionProvider perception,
      ChassisStateProvider chassis,
      SkillTable skills,
      MechanismsConfig mechanismsConfig) {
    this.mechanisms = mechanisms;
    this.setpoints = setpoints;
    this.perception = perception;
    this.chassis = chassis;
    this.skills = skills;

    MechanismConfig turret = mechanismsConfig.get(TURRET).orElse(null);
    if (turret != null && turret.softLimits().isPresent()) {
      this.turretReverseLimitRot = turret.softLimits().get().reverse();
      this.turretForwardLimitRot = turret.softLimits().get().forward();
    } else {
      this.turretReverseLimitRot = kDefaultReverseLimitRot;
      this.turretForwardLimitRot = kDefaultForwardLimitRot;
    }

    // The skill table owns the named source references; resolve them once (safe defaults if the
    // table is invalid — the interpreter then runs safe-idle anyway).
    this.hubShooterSrc = source("shoot", SkillSpec::shooter, "shooter.shotCalc");
    this.hubHoodSrc = source("shoot", SkillSpec::hood, "hood.shotCalc");
    this.passShooterSrc = source("passShoot", SkillSpec::shooter, "shooter.passing");
    this.passHoodSrc = source("passShoot", SkillSpec::hood, "hood.passing");
    this.unjamShooterSrc = source("unjam", SkillSpec::shooter, "shooter.unjamMax");
  }

  private String source(
      String skill, java.util.function.Function<SkillSpec, String> field, String fallback) {
    return skills.get(skill).map(field).filter(s -> s != null && !s.isEmpty()).orElse(fallback);
  }

  /** Re-seed at enable (replaces the legacy onEnable). Manual mode persists (set by the caller). */
  public void onEnable() {
    double turretRot = mechanisms.read(TURRET).positionRot();
    aiming.reset(turretRot, chassis.continuousYawDeg());
    staleness.reset();
    hoodHold.reset();
    clearingTimer.stop();
    clearingTimer.reset();
    phase = Phase.IDLE;
  }

  /** One robot loop. {@code mechanisms.updateInputs()} must have run already this loop. */
  public void periodic(InterpreterInputs inputs) {
    double yawDeg = chassis.continuousYawDeg();
    ChassisSpeeds speeds = chassis.robotRelativeSpeeds();
    double omega = speeds.omegaRadiansPerSecond;
    double turretRot = mechanisms.read(TURRET).positionRot();
    staleness.update(perception.isResultFresh());

    // ── Intake axis: runs in EVERY mode, including manual (mechanism-level, sensor-free). ──
    intake.update(mechanisms, inputs.intakeDeployRequested());
    intakeStatus = inputs.intakeDeployRequested() ? AxisStatus.RUNNING : AxisStatus.OK;

    // ── Manual mode: the scoring axis commands NOTHING; the operator drives mechanisms directly.
    // ──
    if (inputs.manualMode()) {
      if (!prevManualMode) {
        enterManual(turretRot);
      }
      aiming.rebaseToCurrent(turretRot); // W7: no snap-back on exit
      prevManualMode = true;
      phase = Phase.MANUAL;
      activeScoringSkill = "manual";
      scoringStatus = AxisStatus.OK;
      return;
    }
    if (prevManualMode) {
      exitManualToIdle(turretRot, yawDeg);
    }
    prevManualMode = false;

    // ── Safe fallback: an invalid skill table forces safe-idle (everything off/hold). ──
    if (!skills.isValid()) {
      runSafeIdle(turretRot, yawDeg, omega);
      scoringStatus = AxisStatus.FAIL;
      activeScoringSkill = "safe-idle";
      return;
    }

    String skillName = inputs.scoringSkill();
    SkillSpec skill = skills.get(skillName).filter(SkillSpec::isScoring).orElse(null);
    if (skill == null) {
      runSafeIdle(turretRot, yawDeg, omega);
      scoringStatus = AxisStatus.FAIL;
      activeScoringSkill = "safe-idle";
      return;
    }
    activeScoringSkill = skillName;
    scoringStatus = "idle".equals(skillName) ? AxisStatus.OK : AxisStatus.RUNNING;

    // ── Sequence one transition, then execute the phase (legacy sampleConditions + Transitions +
    //    onStateEntry + applyStateBehavior). ──
    Goal goal = goalOf(skillName);
    boolean passSelected = "passAim".equals(skillName) || "passShoot".equals(skillName);
    Conditions conditions =
        new Conditions(
            passSelected,
            aiming.isAimed(),
            shooterAtSpeed(),
            shooterCommandedNonZero(),
            intake.isOut(),
            clearingTimer.hasElapsed(ReflexConstants.kClearingSeconds));

    Phase next = ScoringSequencer.next(phase, goal, conditions);
    if (next != phase) {
      onPhaseEntry(next);
      phase = next;
    }
    applyPhaseBehavior(inputs, turretRot, yawDeg, omega, speeds);
  }

  private static Goal goalOf(String skill) {
    return switch (skill) {
      case "shoot", "passShoot" -> Goal.SHOOT;
      case "passAim" -> Goal.PASS;
      case "unjam" -> Goal.UNJAM;
      default -> Goal.IDLE;
    };
  }

  // One-time actions on entering a phase (legacy onStateEntry, verbatim). The shooter is stopped
  // ONLY on entry into a non-shooting phase — never re-commanded while the phase persists. This is
  // load-bearing: onEnable force-sets IDLE WITHOUT an entry (see onEnable), so a mid-sequence
  // re-enable leaves the flywheel at its last command exactly as the legacy robot did (the disabled
  // lockout reflex is the real-robot safety net for that case).
  private void onPhaseEntry(Phase next) {
    switch (next) {
      case CLEARING -> {
        clearingTimer.restart();
        mechanisms.stop(SHOOTER);
      }
      case IDLE, INTAKING, PASS_AIMING -> mechanisms.stop(SHOOTER);
      default -> {}
    }
  }

  private void applyPhaseBehavior(
      InterpreterInputs inputs,
      double turretRot,
      double yawDeg,
      double omega,
      ChassisSpeeds speeds) {
    // Turret tracking runs in every phase (passing only in the PASS_* phases; CLEARING tracks hub,
    // legacy applyStateBehavior).
    boolean usePassing =
        phase == Phase.PASS_AIMING || phase == Phase.PASS_SPINNING_UP || phase == Phase.PASSING;
    double target =
        usePassing
            ? aiming.trackPassing(
                turretRot, perception, yawDeg, turretReverseLimitRot, turretForwardLimitRot)
            : aiming.trackHub(
                turretRot, perception, yawDeg, omega, turretReverseLimitRot, turretForwardLimitRot);
    mechanisms.setPosition(TURRET, target, 0.0);

    // Staleness → safe (NEW, inert unless the perception pipeline is dead): a firing phase backs
    // the
    // shooter + feed off rather than firing blind. Inert in tests (a fresh frame every loop).
    boolean firing =
        phase == Phase.SPINNING_UP
            || phase == Phase.SHOOTING
            || phase == Phase.PASS_SPINNING_UP
            || phase == Phase.PASSING;
    if (firing && staleness.isStale()) {
      mechanisms.stop(SHOOTER);
      IndexerFeed.rest(mechanisms, false, intake.isOut());
      hoodHold.update(HoodHold.Mode.UNCOMMANDED, mechanisms.read(HOOD).positionRot());
      scoringStatus = AxisStatus.FAIL;
      return;
    }

    SetpointContext ctx =
        new SetpointContext(aiming.getDistanceToHub(), speeds, turretRot * 2.0 * Math.PI);
    switch (phase) {
      case IDLE, INTAKING -> {
        // Shooter stopped on entry only (onPhaseEntry); not re-commanded while resting.
        holdHood();
        IndexerFeed.rest(mechanisms, inputs.manualFeedHeld(), intake.isOut());
      }
      case SPINNING_UP -> {
        mechanisms.setVelocity(SHOOTER, setpoints.evaluate(hubShooterSrc, ctx));
        sourceHood(hubHoodSrc, ctx);
        IndexerFeed.gated(mechanisms, false);
      }
      case SHOOTING -> {
        mechanisms.setVelocity(SHOOTER, setpoints.evaluate(hubShooterSrc, ctx));
        sourceHood(hubHoodSrc, ctx);
        IndexerFeed.gated(mechanisms, true);
      }
      case CLEARING -> {
        // Shooter stopped on entry only (onPhaseEntry); indexers reverse full for kClearingSeconds.
        uncommandedHood();
        IndexerFeed.clearing(mechanisms);
      }
      case PASS_AIMING -> {
        // Shooter stopped on entry only (onPhaseEntry).
        uncommandedHood();
        IndexerFeed.rest(mechanisms, inputs.manualFeedHeld(), intake.isOut());
      }
      case PASS_SPINNING_UP -> {
        mechanisms.setVelocity(SHOOTER, setpoints.evaluate(passShooterSrc, ctx));
        sourceHood(passHoodSrc, ctx);
        IndexerFeed.gated(mechanisms, false);
      }
      case PASSING -> {
        mechanisms.setVelocity(SHOOTER, setpoints.evaluate(passShooterSrc, ctx));
        sourceHood(passHoodSrc, ctx);
        IndexerFeed.gated(mechanisms, true);
      }
      case UNJAMMING -> {
        mechanisms.setVelocity(SHOOTER, setpoints.evaluate(unjamShooterSrc, ctx));
        uncommandedHood();
        IndexerFeed.unjam(mechanisms);
      }
      case MANUAL -> {
        // Unreachable: manual returns before the sequencer runs. Exhaustiveness only.
      }
    }
  }

  private void holdHood() {
    OptionalDouble hold =
        hoodHold.update(HoodHold.Mode.HOLD_CAPTURE, mechanisms.read(HOOD).positionRot());
    hold.ifPresent(rot -> mechanisms.setPosition(HOOD, rot));
  }

  private void uncommandedHood() {
    hoodHold.update(HoodHold.Mode.UNCOMMANDED, mechanisms.read(HOOD).positionRot());
  }

  private void sourceHood(String source, SetpointContext ctx) {
    hoodHold.update(HoodHold.Mode.SOURCE, mechanisms.read(HOOD).positionRot());
    mechanisms.setPosition(HOOD, setpoints.evaluate(source, ctx));
  }

  // ── Predicates (registry) ──

  private boolean shooterAtSpeed() {
    double commanded = mechanisms.commandedRps(SHOOTER);
    double velocity = mechanisms.read(SHOOTER).velocityRps();
    return Math.abs(velocity - commanded) < ReflexConstants.kShooterToleranceRps;
  }

  private boolean shooterCommandedNonZero() {
    return mechanisms.commandedRps(SHOOTER) > 0.0;
  }

  // ── Manual + safe-idle ──

  private void enterManual(double turretRot) {
    mechanisms.stop(SHOOTER);
    mechanisms.stop(IndexerFeed.VERTICAL);
    mechanisms.stop(IndexerFeed.HORIZONTAL);
    mechanisms.stop(IndexerFeed.UPWARD);
    mechanisms.setPosition(TURRET, turretRot, 0.0);
    mechanisms.setPosition(HOOD, mechanisms.read(HOOD).positionRot());
    clearingTimer.stop();
    clearingTimer.reset();
    hoodHold.reset();
  }

  private void exitManualToIdle(double turretRot, double yawDeg) {
    aiming.reset(turretRot, yawDeg); // re-sync to IDLE, never the pre-manual state
    hoodHold.reset();
    clearingTimer.stop();
    clearingTimer.reset();
    mechanisms.stop(SHOOTER);
    mechanisms.stop(IndexerFeed.VERTICAL);
    mechanisms.stop(IndexerFeed.HORIZONTAL);
    mechanisms.stop(IndexerFeed.UPWARD);
    phase = Phase.IDLE;
  }

  // Hardcoded safe behavior for an invalid skill table: turret tracks hub (always cable-limited
  // safe), shooter off, hood holds, all feed stopped.
  private void runSafeIdle(double turretRot, double yawDeg, double omega) {
    double target =
        aiming.trackHub(
            turretRot, perception, yawDeg, omega, turretReverseLimitRot, turretForwardLimitRot);
    mechanisms.setPosition(TURRET, target, 0.0);
    mechanisms.stop(SHOOTER);
    holdHood();
    mechanisms.stop(IndexerFeed.VERTICAL);
    mechanisms.stop(IndexerFeed.HORIZONTAL);
    mechanisms.stop(IndexerFeed.UPWARD);
    phase = Phase.IDLE;
  }

  // ── Accessors for the SkillServer status + tests ──

  public AxisStatus scoringStatus() {
    return scoringStatus;
  }

  public AxisStatus intakeStatus() {
    return intakeStatus;
  }

  public String activeScoringSkill() {
    return activeScoringSkill;
  }

  public Phase phase() {
    return phase;
  }

  public TurretAiming aiming() {
    return aiming;
  }

  public IntakeReflex intakeReflex() {
    return intake;
  }

  public boolean isClearing() {
    return phase == Phase.CLEARING;
  }
}
