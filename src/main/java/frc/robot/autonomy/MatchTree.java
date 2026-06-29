package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.autonomy.ShiftSchedule.Mode;
import frc.robot.autonomy.StrategyConfig.Plan;
import frc.robot.autonomy.bt.Fallback;
import frc.robot.autonomy.bt.Leaf;
import frc.robot.autonomy.bt.Node;
import frc.robot.autonomy.bt.Sequence;
import frc.robot.autonomy.bt.Status;
import frc.robot.runtime.SkillInterpreter.ScoringStatus;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * The shift-aware match brain: a behavior tree that plays the whole teleop on its own, driven by
 * the editable {@code strategy.json} (via {@link StrategyConfig}) and the 2026 SHIFT clock (via
 * {@link ShiftSchedule}). It decides nothing about HOW to shoot/intake — it reuses the existing
 * skills via the skill server — only WHEN and WHERE, and every target it picks is clamped into
 * legal space by {@link LegalRegion} before the navigator sees it.
 *
 * <p>Each loop it derives a {@link Mode} from the teleop clock + the FMS auto-winner game data and
 * dispatches the matching plan:
 *
 * <ul>
 *   <li><b>OUR_HUB_ACTIVE</b> (transition + our active shifts) — shoot + cycle on our side: the
 *       {@link MatchCycle} latch alternates a park-and-shoot with a short collect of the plan's
 *       collect route.
 *   <li><b>OUR_HUB_INACTIVE</b> (the opponent's shift, our hub dead) — harvest deep along the
 *       plan's harvest route, then, within {@code returnLeadSeconds} of our hub re-activating,
 *       stage home so we arrive loaded as our shift starts (and fire immediately — the cycle jumps
 *       to SHOOT).
 *   <li><b>ENDGAME</b> (both hubs active; this robot has no climb) — score-cycle the endgame plan.
 * </ul>
 *
 * <p>Authoring frame is BLUE: the strategy poses are blue-origin and {@link LegalRegion} reasons in
 * the blue frame; the {@link Navigator} alliance-flips the final target for red. AUTO is not
 * handled here — the 20 s auto runs a PathPlanner routine from the chooser.
 */
public final class MatchTree {
  private static final Alliance kAuthoringFrame = Alliance.Blue;

  private final Navigator navigator;
  private final Consumer<String> requestSkill;
  private final Consumer<Boolean> requestIntakeDeploy;
  private final Supplier<ScoringStatus> scoringStatus;
  private final PossessionProvider possession;
  private final BooleanSupplier humanTakeover;
  private final DoubleSupplier teleopTimeRemaining;
  private final Supplier<Optional<Boolean>> ourHubInactiveInShift1;
  private final StrategyConfig strategy;
  private final Supplier<Optional<Mode>> modeOverride;

  private final MatchCycle cycle = new MatchCycle();
  private final Timer roamTimer = new Timer();
  private final Node root;

  // Live telemetry so the brain's state is visible on NetworkTables (table /Autonomy).
  private final StringPublisher modePub;
  private final StringPublisher cyclePhasePub;
  private final DoublePublisher untilActivePub;
  private final BooleanPublisher overridePub;

  // Per-loop derived state (read by the tree's leaf actions/conditions).
  private Mode currentMode = Mode.OUR_HUB_ACTIVE;
  private MatchCycle.Phase lastCyclePhase = MatchCycle.Phase.COLLECT;
  private double secondsUntilActive = 0.0;
  private int collectIndex = 0; // bumps on each COLLECT entry (score modes)
  private int harvestIndex = 0; // bumps on arrival (harvest mode)

  public MatchTree(
      Navigator navigator,
      Consumer<String> requestSkill,
      Consumer<Boolean> requestIntakeDeploy,
      Supplier<ScoringStatus> scoringStatus,
      PossessionProvider possession,
      BooleanSupplier humanTakeover,
      DoubleSupplier teleopTimeRemaining,
      Supplier<Optional<Boolean>> ourHubInactiveInShift1,
      StrategyConfig strategy,
      Supplier<Optional<Mode>> modeOverride) {
    this.navigator = navigator;
    this.requestSkill = requestSkill;
    this.requestIntakeDeploy = requestIntakeDeploy;
    this.scoringStatus = scoringStatus;
    this.possession = possession;
    this.humanTakeover = humanTakeover;
    this.teleopTimeRemaining = teleopTimeRemaining;
    this.ourHubInactiveInShift1 = ourHubInactiveInShift1;
    this.strategy = strategy;
    this.modeOverride = modeOverride;
    NetworkTable t = NetworkTableInstance.getDefault().getTable("Autonomy");
    this.modePub = t.getStringTopic("mode").publish();
    this.cyclePhasePub = t.getStringTopic("cyclePhase").publish();
    this.untilActivePub = t.getDoubleTopic("secondsUntilActive").publish();
    this.overridePub = t.getBooleanTopic("modeOverrideActive").publish();
    this.roamTimer.start();
    this.root = buildRoot();
  }

  /** Tick the whole tree once. Call every robot loop. */
  public Status tick() {
    double t = teleopTimeRemaining.getAsDouble();
    Optional<Boolean> inactiveFirst = ourHubInactiveInShift1.get();
    Mode shiftMode =
        strategy.isValid() ? ShiftSchedule.modeOf(t, inactiveFirst) : Mode.OUR_HUB_ACTIVE;
    // A sim/debug override (dashboard chooser) forces the mode so you can watch each behavior
    // without running a practice match + game-data string; empty = use the real shift clock.
    Optional<Mode> override = modeOverride.get();
    Mode mode = override.orElse(shiftMode);
    // A forced override has no real upcoming re-activation, so never stage early — harvest the
    // whole
    // time. (Otherwise the real clock reads "active now" → secondsUntilActive 0 → it stages and
    // stops instead of going to the neutral zone.)
    secondsUntilActive =
        override.isPresent()
            ? Double.POSITIVE_INFINITY
            : ShiftSchedule.secondsUntilActive(t, inactiveFirst);

    modePub.set(mode.name());
    cyclePhasePub.set(cycle.phase().name());
    untilActivePub.set(secondsUntilActive);
    overridePub.set(override.isPresent());

    if (mode != currentMode) {
      onModeChange(currentMode, mode);
      currentMode = mode;
    }

    // The collect↔shoot latch only runs in the scoring modes; harvest has no shooting.
    if (mode != Mode.OUR_HUB_INACTIVE) {
      if (cycle.phase() != lastCyclePhase) {
        if (cycle.phase() == MatchCycle.Phase.COLLECT) {
          collectIndex++; // next collect visits the next waypoint
        }
        lastCyclePhase = cycle.phase();
      }
      Plan plan = currentPlan();
      // "In region" is the dwell signal: a generous radius around the shoot pose / collection
      // waypoint, so the cycle counts us as collecting/shooting while we roam or divert within it.
      boolean inRegion =
          cycle.inShootPhase()
              ? withinRadius(
                  legalShoot(shootPoseFor(plan)), AutonomyConstants.kShootRegionRadiusMeters)
              : withinRadius(collectCenter(plan), AutonomyConstants.kCollectRegionRadiusMeters);
      cycle.update(possession.isFull(), inRegion, scoringStatus.get() == ScoringStatus.SUCCEEDED);
    }

    return root.tick();
  }

  public void reset() {
    cycle.reset();
    currentMode = Mode.OUR_HUB_ACTIVE;
    lastCyclePhase = MatchCycle.Phase.COLLECT;
    collectIndex = 0;
    harvestIndex = 0;
    roamTimer.restart();
    root.reset();
  }

  /** Stop everything when the command ends. */
  public void onEnd() {
    navigator.stop();
    requestSkill.accept("idle");
    requestIntakeDeploy.accept(false);
  }

  public MatchCycle.Phase cyclePhase() {
    return cycle.phase();
  }

  public Mode mode() {
    return currentMode;
  }

  // ── Tree construction ───────────────────────────────────────────────────

  private Node buildRoot() {
    return new Fallback(
        humanTakeoverBranch(),
        modeBranch(Mode.ENDGAME, this::scoreAction),
        modeBranch(Mode.OUR_HUB_INACTIVE, this::harvestAction),
        modeBranch(Mode.OUR_HUB_ACTIVE, this::scoreAction),
        Leaf.action(this::scoreAction)); // exhaustive safety net
  }

  private Node humanTakeoverBranch() {
    return new Sequence(
        Leaf.condition(humanTakeover),
        Leaf.run(
            () -> {
              navigator.stop();
              requestSkill.accept("idle");
              requestIntakeDeploy.accept(false);
            }));
  }

  private Node modeBranch(Mode mode, Supplier<Status> action) {
    return new Sequence(Leaf.condition(() -> currentMode == mode), Leaf.action(action));
  }

  // ── Mode behaviors ──────────────────────────────────────────────────────

  // Shoot + cycle on our side (OUR_HUB_ACTIVE / ENDGAME). The cycle latch decides shoot vs collect.
  private Status scoreAction() {
    Plan plan = currentPlan();
    if (cycle.inShootPhase()) {
      Pose2d shoot = legalShoot(shootPoseFor(plan));
      navigator.goTo(shoot); // navigator diverts around any opponent sitting on the spot
      // Fire from anywhere in the shoot region — empty the hopper ASAP, don't insist on a point.
      requestSkill.accept(
          withinRadius(shoot, AutonomyConstants.kShootRegionRadiusMeters) ? "shoot" : "idle");
      requestIntakeDeploy.accept(false);
    } else {
      // Roam the collection region to cover the side and gather, rather than parking at a point.
      navigator.goTo(roam(collectCenter(plan), AutonomyConstants.kCollectRoamRadiusMeters));
      requestSkill.accept("idle");
      requestIntakeDeploy.accept(true);
    }
    return Status.RUNNING;
  }

  // Harvest deep along the route; within returnLead of re-activation, stage home loaded and ready.
  private Status harvestAction() {
    Plan plan = currentPlan();
    if (secondsUntilActive <= strategy.returnLeadSeconds()) {
      navigator.goTo(legalNav(stagePoseFor(plan)));
    } else {
      navigator.goTo(harvestTarget(plan));
    }
    requestSkill.accept("idle");
    requestIntakeDeploy.accept(true);
    return Status.RUNNING;
  }

  private void onModeChange(Mode from, Mode to) {
    collectIndex = 0;
    harvestIndex = 0;
    lastCyclePhase = cycle.phase();
    // Re-activation after an off-shift harvest: we staged home loaded, so fire at once.
    if (to == Mode.OUR_HUB_ACTIVE && from == Mode.OUR_HUB_INACTIVE) {
      cycle.startShootPhase();
    }
  }

  // ── Plan resolution (strategy.json, with safe defaults when absent/invalid) ──

  private Plan currentPlan() {
    return strategy.plan(currentMode).orElse(null);
  }

  private Pose2d shootPoseFor(Plan plan) {
    if (plan != null && plan.shootFrom().isPresent()) {
      return strategy.pose(plan.shootFrom().get()).orElse(AutonomyConstants.kShootPose);
    }
    return AutonomyConstants.kShootPose;
  }

  private Pose2d stagePoseFor(Plan plan) {
    if (plan != null && plan.stageAt().isPresent()) {
      return strategy.pose(plan.stageAt().get()).orElse(AutonomyConstants.kShootPose);
    }
    return AutonomyConstants.kShootPose;
  }

  private List<Pose2d> collectRouteFor(Plan plan) {
    if (plan != null) {
      List<Pose2d> route = strategy.route(plan.collectRoute());
      if (!route.isEmpty()) {
        return route;
      }
    }
    return List.of(AutonomyConstants.kCollectionPose);
  }

  private List<Pose2d> harvestRouteFor(Plan plan) {
    if (plan != null) {
      List<Pose2d> route = strategy.route(plan.harvestRoute());
      if (!route.isEmpty()) {
        return route;
      }
    }
    return List.of(AutonomyConstants.kCollectionPose);
  }

  // The collection waypoint we are working this COLLECT phase (advances once per phase entry).
  private Pose2d collectCenter(Plan plan) {
    List<Pose2d> route = collectRouteFor(plan);
    return legalNav(route.get(Math.floorMod(collectIndex, route.size())));
  }

  // A roaming point around {@code center}: a fresh point every kRoamHoldSeconds, spread by the
  // golden angle so the robot sweeps and COVERS the region instead of standing still. Clamped
  // legal.
  private Pose2d roam(Pose2d center, double radius) {
    long step = (long) (roamTimer.get() / AutonomyConstants.kRoamHoldSeconds);
    double angle = step * 2.399963; // golden angle (rad) — successive points spread around
    double r = radius * (0.45 + 0.55 * ((step % 3) / 2.0)); // vary the ring too
    return legalNav(
        new Pose2d(
            center.getX() + r * Math.cos(angle),
            center.getY() + r * Math.sin(angle),
            center.getRotation()));
  }

  // Harvest target: cycle the route, advancing to the next waypoint each time we arrive.
  private Pose2d harvestTarget(Plan plan) {
    List<Pose2d> route = harvestRouteFor(plan);
    Pose2d target = legalNav(route.get(Math.floorMod(harvestIndex, route.size())));
    if (navigator.atTarget()) {
      harvestIndex++;
    }
    return target;
  }

  // ── Legality + geometry helpers ─────────────────────────────────────────

  private Pose2d legalShoot(Pose2d pose) {
    return LegalRegion.sanitizeShoot(pose, kAuthoringFrame);
  }

  private Pose2d legalNav(Pose2d pose) {
    return new Pose2d(
        LegalRegion.sanitize(pose.getTranslation(), kAuthoringFrame), pose.getRotation());
  }

  private boolean withinRadius(Pose2d pose, double radius) {
    return navigator.pose().getTranslation().getDistance(pose.getTranslation()) < radius;
  }
}
