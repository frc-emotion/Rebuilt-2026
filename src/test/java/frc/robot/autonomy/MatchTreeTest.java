package frc.robot.autonomy;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.runtime.ChassisStateProvider;
import frc.robot.runtime.Mechanisms;
import frc.robot.runtime.Setpoints;
import frc.robot.runtime.SkillInterpreter;
import frc.robot.runtime.SkillServer;
import frc.robot.runtime.config.MechanismsConfig;
import frc.robot.runtime.config.SkillTable;
import frc.robot.runtime.perception.VisionPerception;
import frc.robot.runtime.reflex.ScoringSequencer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionPoseEstimator;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * The shift-aware autonomy acceptance test: the strategy-driven behavior tree, driving the REAL
 * skill runtime (mechanisms in sim + interpreter + skill server) with a fake navigator, plays each
 * match mode with ZERO human input. Movement is faked (the navigator seam) so this tests the
 * DECISION layer deterministically without depending on full swerve-sim physics; the real
 * PathPlannerNavigator is exercised in {@link AutonomySimObservationTest}. Asserts: (1) in an
 * active shift it runs ≥2 collect→shoot cycles, spins the flywheel while shooting, deploys the
 * intake while collecting, and reaches the strategy shoot pose; (2) in an inactive shift it
 * harvests deep into the neutral zone and never fires at the dead hub; (3) when our hub
 * re-activates it jumps straight to the SHOOT phase (staged loaded).
 */
class MatchTreeTest {

  /** A kinematic fake: moves the stored pose toward the target each step; no physics. */
  private static final class FakeNavigator implements Navigator {
    private Pose2d pose;
    private Pose2d target;
    private static final double kSpeedMps = 4.0;
    private static final double kDt = 0.02;

    FakeNavigator(Pose2d start) {
      this.pose = start;
    }

    @Override
    public void goTo(Pose2d target) {
      this.target = target;
    }

    @Override
    public boolean atTarget() {
      return target != null && pose.getTranslation().getDistance(target.getTranslation()) < 0.1;
    }

    @Override
    public void stop() {
      target = null;
    }

    @Override
    public Pose2d pose() {
      return pose;
    }

    void step() {
      if (target == null) {
        return;
      }
      Translation2d to = target.getTranslation().minus(pose.getTranslation());
      double dist = to.getNorm();
      if (dist < 1e-6) {
        return;
      }
      double stepDist = Math.min(dist, kSpeedMps * kDt);
      pose =
          new Pose2d(pose.getTranslation().plus(to.times(stepDist / dist)), target.getRotation());
    }
  }

  private static final class FakeVisionIO implements VisionIO {
    private static final Transform3d kCamToTag =
        new Transform3d(new Translation3d(2.5, 0.2, 0.0), new Rotation3d());
    int tagId = 18; // hub tag visible so the shoot skill can aim
    private long frame = 0;

    @Override
    public VisionIOInputs updateInputs() {
      frame++;
      long ts = frame * 20_000;
      List<TargetCorner> corners =
          List.of(
              new TargetCorner(0, 0),
              new TargetCorner(1, 0),
              new TargetCorner(1, 1),
              new TargetCorner(0, 1));
      List<PhotonTrackedTarget> targets =
          tagId < 0
              ? List.of()
              : List.of(
                  new PhotonTrackedTarget(
                      0.0, 0.0, 1.0, 0.0, tagId, -1, 0.0f, kCamToTag, kCamToTag, 0.05, corners,
                      corners));
      return new VisionIOInputs(
          true, List.of(new PhotonPipelineResult(frame, ts, ts + 1000, 0, targets)));
    }
  }

  private static Drive drive;

  private Mechanisms mechanisms;
  private Vision vision;
  private SkillInterpreter interpreter;
  private SkillServer server;
  private FakeNavigator nav;
  private StrategyConfig strategy;
  private MatchTree tree;

  // Test-controllable match signals (the suppliers read these live).
  private double teleopTime = 135.0; // TRANSITION (both hubs active) by default
  private Optional<Boolean> inactiveFirst = Optional.empty();
  private Optional<ShiftSchedule.Mode> modeOverride = Optional.empty();

  @BeforeAll
  static void once() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    drive = new Drive();
  }

  @AfterAll
  static void done() {
    SimHooks.resumeTiming();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @BeforeEach
  void build() {
    CommandScheduler.getInstance().unregisterAllSubsystems();
    teleopTime = 135.0;
    inactiveFirst = Optional.empty();
    modeOverride = Optional.empty();
    FakeVisionIO visionIo = new FakeVisionIO();
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    MechanismsConfig mechConfig =
        MechanismsConfig.load(Filesystem.getDeployDirectory().toPath().resolve("mechanisms.json"));
    mechanisms = new Mechanisms(mechConfig, false);
    vision =
        new Vision(
            visionIo,
            new VisionPoseEstimator(layout, (p, t, s) -> {}),
            () -> mechanisms.read("turret").positionRot(),
            () -> mechanisms.read("turret").velocityRps());
    ChassisStateProvider chassis =
        new ChassisStateProvider() {
          @Override
          public double continuousYawDeg() {
            return drive.getContinuousYawDeg();
          }

          @Override
          public ChassisSpeeds robotRelativeSpeeds() {
            return drive.getRobotRelativeSpeeds();
          }
        };
    interpreter =
        new SkillInterpreter(
            mechanisms,
            new Setpoints(),
            new VisionPerception(vision),
            chassis,
            SkillTable.load(Filesystem.getDeployDirectory().toPath().resolve("skills.json")),
            mechConfig);
    server = new SkillServer();
    strategy =
        StrategyConfig.load(Filesystem.getDeployDirectory().toPath().resolve("strategy.json"));
    nav = new FakeNavigator(new Pose2d(2.6, 4.035, Rotation2d.kZero));
    tree =
        new MatchTree(
            nav,
            server::invoke,
            server::setIntakeDeploy,
            () -> interpreter.scoringStatus(),
            PossessionProvider.UNKNOWN,
            () -> false, // no human takeover
            () -> teleopTime,
            () -> inactiveFirst,
            strategy,
            () -> modeOverride);
    interpreter.onEnable();
  }

  /** One full robot loop: decide → move the fake pose → run the real runtime. */
  private void loop() {
    tree.tick();
    nav.step();
    vision.periodic();
    mechanisms.updateInputs();
    interpreter.periodic(server.resolve());
    SimHooks.stepTiming(0.02);
  }

  @Test
  void activeShiftRunsCollectShootCyclesAndFires() {
    assertTrue(strategy.isValid(), "deploy/strategy.json must load for this test");
    teleopTime = 135.0; // TRANSITION → OUR_HUB_ACTIVE

    Pose2d shootPose = strategy.pose("shoot_main").orElseThrow();
    Pose2d collectPose = strategy.pose("harvest_left").orElseThrow();

    MatchCycle.Phase prev = tree.cyclePhase();
    int collectToShoot = 0;
    boolean shooterSpunWhileShooting = false;
    boolean intakeOutWhileCollecting = false;
    boolean reachedShootPose = false;
    boolean reachedCollect = false;

    for (int i = 0; i < 2500; i++) {
      loop();
      MatchCycle.Phase phase = tree.cyclePhase();
      if (prev == MatchCycle.Phase.COLLECT && phase == MatchCycle.Phase.SHOOT) {
        collectToShoot++;
      }
      prev = phase;
      if (phase == MatchCycle.Phase.SHOOT && mechanisms.commandedRps("shooter") > 0.0) {
        shooterSpunWhileShooting = true;
      }
      if (phase == MatchCycle.Phase.COLLECT && server.localIntakeDeploy()) {
        intakeOutWhileCollecting = true;
      }
      if (nav.pose().getTranslation().getDistance(shootPose.getTranslation()) < 0.3) {
        reachedShootPose = true;
      }
      if (nav.pose().getTranslation().getDistance(collectPose.getTranslation()) < 0.8) {
        reachedCollect = true;
      }
    }

    assertTrue(
        collectToShoot >= 2, "expected >=2 full collect->shoot cycles, got " + collectToShoot);
    assertTrue(shooterSpunWhileShooting, "the shoot skill must spin the flywheel while shooting");
    assertTrue(intakeOutWhileCollecting, "the intake must be deployed while collecting");
    assertTrue(reachedShootPose, "the robot must reach the strategy shoot pose");
    assertTrue(reachedCollect, "the robot must reach a strategy collect waypoint");
  }

  @Test
  void inactiveShiftHarvestsNeutralZoneAndNeverFires() {
    inactiveFirst = Optional.of(true); // our hub inactive in SHIFT 1
    teleopTime = 120.0; // SHIFT 1 → OUR_HUB_INACTIVE (and >returnLead from re-activation)

    boolean everRequestedShoot = false;
    boolean intakeOutWhileHarvesting = false;
    double maxX = 0.0;

    for (int i = 0; i < 1500; i++) {
      loop();
      String requested = server.resolve().scoringSkill();
      if ("shoot".equals(requested) || "passShoot".equals(requested)) {
        everRequestedShoot = true;
      }
      if (server.localIntakeDeploy()) {
        intakeOutWhileHarvesting = true;
      }
      maxX = Math.max(maxX, nav.pose().getX());
    }

    assertTrue(maxX > 5.0, "harvest must push deep into the neutral zone, got maxX=" + maxX);
    assertTrue(intakeOutWhileHarvesting, "the intake must be deployed while harvesting");
    assertTrue(everRequestedShoot == false, "must never fire at an inactive hub");
  }

  @Test
  void shootWindowFeedsForTheConfiguredEmptyDuration() {
    teleopTime = 135.0; // OUR_HUB_ACTIVE

    // Advance to the first shoot window.
    int guard = 0;
    while (tree.cyclePhase() != MatchCycle.Phase.SHOOT && guard++ < 3000) {
      loop();
    }
    assertTrue(tree.cyclePhase() == MatchCycle.Phase.SHOOT, "must reach a shoot window");

    // Count feed-gate-open (SHOOTING) loops across this whole shoot window.
    int feedingLoops = 0;
    int windowLoops = 0;
    while (tree.cyclePhase() == MatchCycle.Phase.SHOOT && windowLoops++ < 1000) {
      if (interpreter.phase() == ScoringSequencer.Phase.SHOOTING) {
        feedingLoops++;
      }
      loop();
    }

    double fedSeconds = feedingLoops * 0.02;
    double windowSeconds = windowLoops * 0.02;
    assertTrue(
        fedSeconds >= AutonomyConstants.kShootEmptySeconds - 0.5,
        "must keep feeding ~"
            + AutonomyConstants.kShootEmptySeconds
            + "s to empty the hopper, fed "
            + fedSeconds
            + "s");
    assertTrue(
        windowSeconds <= AutonomyConstants.kPhaseHardTimeoutSeconds + 0.5,
        "the shoot window must end (hard cap), lasted " + windowSeconds + "s");
  }

  @Test
  void forcedInactiveOverrideHarvestsInsteadOfStaging() {
    // Sim/debug: force INACTIVE with no real match clock (teleop time < 0). The robot must HARVEST
    // into the neutral zone, not stage home and stop (the override has no upcoming re-activation).
    teleopTime = -1.0;
    modeOverride = Optional.of(ShiftSchedule.Mode.OUR_HUB_INACTIVE);

    double maxX = 0.0;
    for (int i = 0; i < 1500; i++) {
      loop();
      maxX = Math.max(maxX, nav.pose().getX());
    }
    assertTrue(maxX > 5.0, "forced INACTIVE must harvest into the neutral zone, got maxX=" + maxX);
  }

  @Test
  void reActivationJumpsStraightToShootPhase() {
    inactiveFirst = Optional.of(true);
    teleopTime = 108.0; // SHIFT 1, ~3 s left → within returnLead → staging home
    for (int i = 0; i < 200; i++) {
      loop();
    }
    // Our hub re-activates at the start of SHIFT 2.
    teleopTime = 104.0; // SHIFT 2 → OUR_HUB_ACTIVE
    loop();
    assertTrue(
        tree.cyclePhase() == MatchCycle.Phase.SHOOT,
        "re-activation after harvest must jump straight to the SHOOT phase (staged loaded)");
  }
}
