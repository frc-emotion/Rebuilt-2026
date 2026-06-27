package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.autonomy.bt.Fallback;
import frc.robot.autonomy.bt.Leaf;
import frc.robot.autonomy.bt.Node;
import frc.robot.autonomy.bt.Sequence;
import frc.robot.autonomy.bt.Status;
import frc.robot.runtime.SkillInterpreter.ScoringStatus;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * The v1 match autonomy: a behavior tree that plays the whole match on its own with what exists
 * today (skills from the migration + PathPlanner pathfinding). It decides nothing about HOW to
 * shoot/intake — it reuses the existing skills via the skill server — only WHEN and WHERE.
 *
 * <p>Root is a Fallback ticked each loop, in priority order:
 *
 * <ol>
 *   <li><b>human-takeover gate</b> (stub) — if a human has taken over, stop and idle.
 *   <li><b>endgame branch</b> — in the last seconds, park (stub pose; this robot has no climb).
 *   <li><b>collect↔shoot cycle</b> — if the {@link MatchCycle} says shoot: drive to a safe shoot
 *       pose, then run the shoot skill (we can score from anywhere on our side — no
 *       aim-on-the-move); else: drive to the collection region with the intake out and dwell/sweep.
 * </ol>
 *
 * <p>The cycle latch ({@link MatchCycle}) supplies hysteresis/min-dwell so collect and shoot don't
 * thrash. Movement is the {@link Navigator} seam (PathPlanner pathfinding in production, a fake in
 * tests). Possession / opponent-obstacle / learned-nav-cost are all empty seams today.
 */
public final class MatchTree {
  private final Navigator navigator;
  private final Consumer<String> requestSkill;
  private final Consumer<Boolean> requestIntakeDeploy;
  private final Supplier<ScoringStatus> scoringStatus;
  private final PossessionProvider possession;
  private final BooleanSupplier humanTakeover;
  private final DoubleSupplier matchTimeRemaining;

  private final MatchCycle cycle = new MatchCycle();
  private final Timer sweepTimer = new Timer();
  private final Node root;

  public MatchTree(
      Navigator navigator,
      Consumer<String> requestSkill,
      Consumer<Boolean> requestIntakeDeploy,
      Supplier<ScoringStatus> scoringStatus,
      PossessionProvider possession,
      BooleanSupplier humanTakeover,
      DoubleSupplier matchTimeRemaining) {
    this.navigator = navigator;
    this.requestSkill = requestSkill;
    this.requestIntakeDeploy = requestIntakeDeploy;
    this.scoringStatus = scoringStatus;
    this.possession = possession;
    this.humanTakeover = humanTakeover;
    this.matchTimeRemaining = matchTimeRemaining;
    this.sweepTimer.restart();
    this.root = buildRoot();
  }

  /** Tick the whole tree once. Call every robot loop. */
  public Status tick() {
    cycle.update(possession.isFull(), scoringStatus.get() == ScoringStatus.SUCCEEDED);
    return root.tick();
  }

  public void reset() {
    cycle.reset();
    sweepTimer.restart();
    root.reset();
  }

  /** Stop everything when the autonomy command ends. */
  public void onEnd() {
    navigator.stop();
    requestSkill.accept("idle");
    requestIntakeDeploy.accept(false);
  }

  public MatchCycle.Phase cyclePhase() {
    return cycle.phase();
  }

  // ── Tree construction ──────────────────────────────────────────────────

  private Node buildRoot() {
    return new Fallback(humanTakeoverBranch(), endgameBranch(), collectShootCycle());
  }

  // Highest priority: if a human has taken over, the tree does nothing but hold safe.
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

  // Endgame: park at a stub pose (no climb on this robot). Inert in sim, where match time is < 0.
  private Node endgameBranch() {
    return new Sequence(
        Leaf.condition(
            () -> {
              double t = matchTimeRemaining.getAsDouble();
              return t > 0.0 && t <= AutonomyConstants.kEndgameSeconds;
            }),
        travelTo(() -> AutonomyConstants.kEndgameParkPose, false),
        Leaf.action(
            () -> {
              requestSkill.accept("idle");
              return Status.RUNNING;
            }));
  }

  // The main loop: shoot if the cycle says so, otherwise collect.
  private Node collectShootCycle() {
    return new Fallback(shootBranch(), collectBranch());
  }

  // SHOOT: only when the cycle latch is in the shoot phase. Drive to the shoot pose (idle while
  // traveling), then hold the shoot skill. We can score from anywhere on our side, so this is just
  // "park at a safe pose and shoot" — no aiming on the move (the turret auto-aims via the skill).
  private Node shootBranch() {
    return new Sequence(
        Leaf.condition(cycle::inShootPhase),
        travelTo(() -> AutonomyConstants.kShootPose, false),
        Leaf.action(
            () -> {
              requestSkill.accept("shoot");
              requestIntakeDeploy.accept(false);
              return Status.RUNNING; // hold shooting; the cycle decides when we leave
            }));
  }

  // COLLECT: drive to the (sweeping) collection region with the intake out, then dwell/sweep there.
  private Node collectBranch() {
    return new Sequence(
        travelTo(this::sweepTarget, true),
        Leaf.action(
            () -> {
              requestSkill.accept("idle");
              requestIntakeDeploy.accept(true);
              return Status.RUNNING; // hold collecting; the cycle's dwell decides when we leave
            }));
  }

  // Travel = set the desired skill/intake for the trip, then drive there. Returns SUCCESS only once
  // the DriveTo arrives, so the branch's terminal action runs only after we are parked.
  private Node travelTo(Supplier<Pose2d> target, boolean intakeOutWhileTraveling) {
    return new Sequence(
        Leaf.run(
            () -> {
              requestSkill.accept("idle");
              requestIntakeDeploy.accept(intakeOutWhileTraveling);
            }),
        new DriveToNode(navigator, target));
  }

  // The collection sweep: with no game-piece tracking, oscillate the target laterally so the robot
  // wiggles across the collection region to gather pieces. A documented stand-in for a real sweep.
  private Pose2d sweepTarget() {
    Pose2d base = AutonomyConstants.kCollectionPose;
    boolean second =
        ((long) (sweepTimer.get() / AutonomyConstants.kSweepHalfPeriodSeconds)) % 2 == 1;
    double dy = (second ? 1.0 : -1.0) * (AutonomyConstants.kSweepLateralMeters / 2.0);
    return new Pose2d(new Translation2d(base.getX(), base.getY() + dy), base.getRotation());
  }
}
