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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionPoseEstimator;
import java.util.List;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * The v1 autonomy acceptance test: the behavior tree, driving the REAL skill runtime (mechanisms in
 * sim + interpreter + skill server) with a fake navigator, runs repeated collect→shoot cycles with
 * ZERO human input. Movement is faked (the navigator seam) so this tests the DECISION layer
 * deterministically without depending on full swerve-sim physics; the real PathPlannerNavigator is
 * exercised in simulateJava. Asserts ≥2 full cycles, that the shoot skill actually spins the
 * flywheel while shooting, and that the intake deploys while collecting.
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
  private MatchTree tree;

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
    nav = new FakeNavigator(new Pose2d(2.0, 3.0, Rotation2d.kZero));
    tree =
        new MatchTree(
            nav,
            server::invoke,
            server::setIntakeDeploy,
            () -> interpreter.scoringStatus(),
            PossessionProvider.UNKNOWN,
            () -> false, // no human takeover
            () -> -1.0); // no match time -> endgame branch inert
    interpreter.onEnable();
  }

  @Test
  void runsRepeatedCollectShootCyclesWithZeroHumanInput() {
    MatchCycle.Phase prev = tree.cyclePhase();
    int collectToShoot = 0;
    boolean shooterSpunWhileShooting = false;
    boolean intakeOutWhileCollecting = false;
    boolean reachedShootPose = false;
    boolean reachedCollectPose = false;

    for (int i = 0; i < 2500; i++) {
      tree.tick(); // decide + request skill + command the navigator
      nav.step(); // advance the faked pose toward the target
      vision.periodic();
      mechanisms.updateInputs();
      interpreter.periodic(server.resolve()); // execute the requested skill on the real runtime
      SimHooks.stepTiming(0.02);

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
      if (nav.pose().getTranslation().getDistance(AutonomyConstants.kShootPose.getTranslation())
          < 0.3) {
        reachedShootPose = true;
      }
      if (nav.pose()
              .getTranslation()
              .getDistance(AutonomyConstants.kCollectionPose.getTranslation())
          < 0.8) {
        reachedCollectPose = true;
      }
    }

    assertTrue(
        collectToShoot >= 2, "expected >=2 full collect->shoot cycles, got " + collectToShoot);
    assertTrue(shooterSpunWhileShooting, "the shoot skill must spin the flywheel while shooting");
    assertTrue(intakeOutWhileCollecting, "the intake must be deployed while collecting");
    assertTrue(reachedShootPose, "the robot must reach the shoot pose");
    assertTrue(reachedCollectPose, "the robot must reach the collection region");
  }
}
