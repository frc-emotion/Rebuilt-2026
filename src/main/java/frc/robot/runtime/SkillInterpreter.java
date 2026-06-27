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

  /**
   * The simple intake-axis status (the intake is a held mechanism toggle, never a goal that fails).
   */
  public enum AxisStatus {
    OK,
    RUNNING,
    FAIL
  }

  /**
   * The richer SCORING-axis status reported to the orchestration brain (NT + getters). It is PURE
   * INSTRUMENTATION: it reflects what the sequencer + reflexes are doing; it never changes a phase
   * transition or a mechanism command.
   *
   * <ul>
   *   <li>{@code IDLE} — no scoring request (resting at IDLE/INTAKING) or manual mode.
   *   <li>{@code RUNNING} — a skill is actively working toward its objective (a firing skill
   *       spinning up / aiming within its timeout, or a held unjam/passAim).
   *   <li>{@code SUCCEEDED} — a firing skill reached the productive feeding phase
   *       (SHOOTING/PASSING). CAVEAT: this robot has NO game-piece sensor (the indexer has no ball
   *       count), so SUCCEEDED means "the feed gate opened and the robot is feeding/firing" — the
   *       best available proxy for a shot. It CANNOT confirm a ball physically launched.
   *   <li>{@code FAILED} — a safe-fallback / dead-pipeline condition: an invalid skill table, an
   *       unknown/non-scoring skill request, or perception stale beyond threshold while a firing
   *       skill was trying to fire (the staleness reflex has already backed the shooter + feed
   *       off).
   *   <li>{@code BLOCKED} — a firing skill exceeded its {@code timeoutSeconds} (skills.json) while
   *       still not feeding. Status only — the sequencer keeps trying; the brain may re-plan.
   * </ul>
   */
  public enum ScoringStatus {
    IDLE,
    RUNNING,
    SUCCEEDED,
    FAILED,
    BLOCKED
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
  // Wall-clock timer for the per-skill BLOCKED timeout: started when a firing skill enters a
  // non-productive (spin-up/aim) phase, reset when it begins feeding, the skill changes, or on
  // enable/manual. Status only — it never touches the sequencer or a mechanism command.
  private final Timer timeoutTimer = new Timer();
  private String timeoutTrackedSkill = "";
  private Phase phase = Phase.IDLE;
  private boolean prevManualMode = false;

  private ScoringStatus scoringStatus = ScoringStatus.IDLE;
  private String scoringReason = "idle";
  private boolean scoringDone = false;
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
    resetTimeout();
    phase = Phase.IDLE;
    scoringStatus = ScoringStatus.IDLE;
    scoringReason = "idle";
    scoringDone = false;
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
      scoringStatus = ScoringStatus.IDLE;
      scoringReason = "manual";
      scoringDone = false;
      resetTimeout();
      return;
    }
    if (prevManualMode) {
      exitManualToIdle(turretRot, yawDeg);
    }
    prevManualMode = false;

    // ── Safe fallback: an invalid skill table forces safe-idle (everything off/hold). ──
    if (!skills.isValid()) {
      runSafeIdle(turretRot, yawDeg, omega);
      failScoring("safe-idle", "invalid skill table");
      return;
    }

    String skillName = inputs.scoringSkill();
    SkillSpec skill = skills.get(skillName).filter(SkillSpec::isScoring).orElse(null);
    if (skill == null) {
      runSafeIdle(turretRot, yawDeg, omega);
      failScoring("safe-idle", "unknown scoring skill '" + skillName + "'");
      return;
    }
    activeScoringSkill = skillName;

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

    // Status instrumentation only (after the phase + mechanism commands are settled this loop).
    manageTimeout(skillName);
    updateScoringStatus(skillName);
    scoringDone =
        skills.get(skillName).map(s -> DonePredicates.evaluate(s.done(), phase)).orElse(false);
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
      // Effect unchanged: back the shooter + feed off rather than fire blind. The FAILED status is
      // reported by updateScoringStatus (which re-derives this same firing+stale condition).
      mechanisms.stop(SHOOTER);
      IndexerFeed.rest(mechanisms, false, intake.isOut());
      hoodHold.update(HoodHold.Mode.UNCOMMANDED, mechanisms.read(HOOD).positionRot());
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

  // ── Status instrumentation (pure; no mechanism or sequencer side effects) ──

  private static boolean isFiringSkill(String skill) {
    return "shoot".equals(skill) || "passShoot".equals(skill);
  }

  private double timeoutOf(String skill) {
    return skills.get(skill).map(SkillSpec::timeoutSeconds).orElse(Double.POSITIVE_INFINITY);
  }

  private void resetTimeout() {
    timeoutTrackedSkill = "";
    timeoutTimer.stop();
    timeoutTimer.reset();
  }

  /**
   * Track wall-clock time the firing skill spends in a non-productive (spin-up/aim) phase. Started
   * fresh when such a phase is first entered for a skill, left running while it persists, and reset
   * the moment feeding begins, the skill changes, or the phase leaves the non-productive set. This
   * sets no mechanism command and is invisible to the sequencer.
   */
  private void manageTimeout(String skill) {
    boolean nonProductive =
        isFiringSkill(skill)
            && (phase == Phase.SPINNING_UP
                || phase == Phase.PASS_SPINNING_UP
                || phase == Phase.PASS_AIMING);
    if (nonProductive && skill.equals(timeoutTrackedSkill)) {
      return; // already counting for this skill
    }
    if (nonProductive) {
      timeoutTrackedSkill = skill;
      timeoutTimer.restart();
    } else {
      resetTimeout();
    }
  }

  private boolean timeoutExceeded(String skill) {
    double t = timeoutOf(skill);
    return Double.isFinite(t) && skill.equals(timeoutTrackedSkill) && timeoutTimer.hasElapsed(t);
  }

  private void failScoring(String activeSkill, String reason) {
    activeScoringSkill = activeSkill;
    scoringStatus = ScoringStatus.FAILED;
    scoringReason = reason;
    scoringDone = false;
    resetTimeout();
  }

  // Map the current phase + skill to the richer scoring status (see ScoringStatus javadoc).
  private void updateScoringStatus(String skill) {
    boolean firingPhase =
        phase == Phase.SPINNING_UP
            || phase == Phase.SHOOTING
            || phase == Phase.PASS_SPINNING_UP
            || phase == Phase.PASSING;
    if (firingPhase && staleness.isStale()) {
      scoringStatus = ScoringStatus.FAILED;
      scoringReason = "perception stale — firing aborted";
      return;
    }
    if ("idle".equals(skill)) {
      scoringStatus = ScoringStatus.IDLE;
      scoringReason = "idle";
      return;
    }
    if (isFiringSkill(skill)) {
      if (phase == Phase.SHOOTING || phase == Phase.PASSING) {
        scoringStatus = ScoringStatus.SUCCEEDED;
        scoringReason = "feeding (feed-gate-open proxy; no game-piece sensor)";
        return;
      }
      if (timeoutExceeded(skill)) {
        scoringStatus = ScoringStatus.BLOCKED;
        scoringReason = "no feed within " + timeoutOf(skill) + "s";
        return;
      }
      scoringStatus = ScoringStatus.RUNNING;
      scoringReason = "spinning up / aiming";
      return;
    }
    // Held non-firing skill (passAim / unjam): active, never times out.
    scoringStatus = ScoringStatus.RUNNING;
    scoringReason = skill + " active";
  }

  // ── Accessors for the SkillServer status + tests ──

  public ScoringStatus scoringStatus() {
    return scoringStatus;
  }

  /** Human-readable reason behind the current scoring status (for the dashboard / brain logs). */
  public String scoringReason() {
    return scoringReason;
  }

  /**
   * The active skill's "done" predicate result (advisory only — held skills are NOT auto-canceled).
   * See {@link DonePredicates}; "feeding" is the feed-gate-open proxy for a shot (no ball sensor).
   */
  public boolean isDone() {
    return scoringDone;
  }

  /**
   * Seconds remaining before the active firing skill is reported BLOCKED, or {@link
   * Double#POSITIVE_INFINITY} when no timeout is being tracked (not firing / no configured
   * timeout).
   */
  public double timeoutRemainingSeconds() {
    if (timeoutTrackedSkill.isEmpty()) {
      return Double.POSITIVE_INFINITY;
    }
    double t = timeoutOf(timeoutTrackedSkill);
    return Double.isFinite(t) ? Math.max(0.0, t - timeoutTimer.get()) : Double.POSITIVE_INFINITY;
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
