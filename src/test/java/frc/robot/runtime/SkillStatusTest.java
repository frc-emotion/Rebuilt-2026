package frc.robot.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.runtime.SkillInterpreter.InterpreterInputs;
import frc.robot.runtime.SkillInterpreter.ScoringStatus;
import frc.robot.runtime.config.MechanismsConfig;
import frc.robot.runtime.config.SkillTable;
import frc.robot.runtime.perception.VisionPerception;
import frc.robot.runtime.reflex.ScoringSequencer.Phase;
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
 * The status + timeout instrumentation catalog: drives the same physics-sim mechanism layer and
 * synthetic camera frames as {@link SkillInterpreterFlowTest}, but asserts on the richer {@link
 * ScoringStatus} the orchestration brain reads — not on phase transitions or mechanism setpoints
 * (those are PURE INSTRUMENTATION and proven untouched by the flow test). Covers SUCCEEDED on feed,
 * the per-skill BLOCKED timeout, IDLE at rest, and FAILED on an unknown skill.
 */
class SkillStatusTest {

  private static final class FakeVisionIO implements VisionIO {
    private static final Transform3d kCamToTag =
        new Transform3d(new Translation3d(2.5, 0.2, 0.0), new Rotation3d());
    int tagId = -1;
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

  private FakeVisionIO fakeVisionIo;
  private Vision vision;
  private Mechanisms mechanisms;
  private SkillInterpreter interpreter;

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
    fakeVisionIo = new FakeVisionIO();
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    mechanisms =
        new Mechanisms(
            MechanismsConfig.load(
                Filesystem.getDeployDirectory().toPath().resolve("mechanisms.json")),
            false);
    vision =
        new Vision(
            fakeVisionIo,
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
            MechanismsConfig.load(
                Filesystem.getDeployDirectory().toPath().resolve("mechanisms.json")));
    fakeVisionIo.tagId = -1;
  }

  private void tick(String skill, int loops) {
    for (int i = 0; i < loops; i++) {
      vision.periodic();
      mechanisms.updateInputs();
      interpreter.periodic(new InterpreterInputs(skill, false, false, false));
      SimHooks.stepTiming(0.02);
    }
  }

  private void settle() {
    interpreter.onEnable();
    tick("idle", 5);
    assertEquals(Phase.IDLE, interpreter.phase());
  }

  @Test
  void idleReportsIdle() {
    settle();
    tick("idle", 20);
    assertEquals(ScoringStatus.IDLE, interpreter.scoringStatus());
    assertFalse(interpreter.isDone(), "idle is never done");
  }

  @Test
  void shootWithTagReachesSucceededOnceFeeding() {
    settle();
    fakeVisionIo.tagId = 18;
    tick("shoot", 460);
    assertEquals(Phase.SHOOTING, interpreter.phase());
    assertEquals(ScoringStatus.SUCCEEDED, interpreter.scoringStatus());
    // The "feeding" done predicate (feed-gate-open proxy; this robot has no game-piece sensor).
    assertTrue(interpreter.isDone(), "feeding satisfies the shoot done predicate");
  }

  @Test
  void shootWithoutTagRunsThenBlocksAfterTimeout() {
    settle();
    // No tag -> never aimed -> the feed gate never opens; the skill spins up forever.
    tick("shoot", 5);
    assertEquals(Phase.SPINNING_UP, interpreter.phase());
    assertEquals(ScoringStatus.RUNNING, interpreter.scoringStatus(), "within the 3.0 s timeout");
    assertFalse(interpreter.isDone());
    // Cross the 3.0 s skills.json timeout (5 + 200 loops = 4.1 s of spin-up).
    tick("shoot", 200);
    assertEquals(Phase.SPINNING_UP, interpreter.phase(), "sequencer unaffected — still trying");
    assertEquals(ScoringStatus.BLOCKED, interpreter.scoringStatus());
    assertFalse(interpreter.isDone(), "blocked is not done");
  }

  @Test
  void unknownSkillReportsFailed() {
    settle();
    tick("bogus", 5);
    assertEquals(ScoringStatus.FAILED, interpreter.scoringStatus());
    assertEquals("safe-idle", interpreter.activeScoringSkill());
    assertFalse(interpreter.isDone());
  }
}
