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
import frc.robot.runtime.config.MechanismsConfig;
import frc.robot.runtime.config.SkillTable;
import frc.robot.runtime.perception.VisionPerception;
import frc.robot.runtime.reflex.IndexerFeed;
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
 * The permanent behavior catalog for the skill-server runtime: every goal flow end-to-end against
 * the physics-sim mechanism layer and synthetic camera frames, the atomic CLEARING preemption
 * matrix, manual mode, and the W12 zero-setpoint feed-gate guard. Assertions are on the interpreter
 * phase + the actual setpoints commanded to the generic mechanism layer (the successor to
 * SuperstructureFlowTest). Per-tick equivalence to the legacy robot was proven by the Phase-6
 * EquivalenceHarnessTest before cutover.
 */
class SkillInterpreterFlowTest {

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

  private boolean shoot;
  private boolean pass;
  private boolean unjam;
  private boolean manualFeed;
  private boolean manualMode;
  private boolean intakeDeploy;

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
    shoot = pass = unjam = manualFeed = manualMode = intakeDeploy = false;
    fakeVisionIo.tagId = -1;
  }

  private String skill() {
    if (unjam) {
      return "unjam";
    }
    if (shoot && pass) {
      return "passShoot";
    }
    if (shoot) {
      return "shoot";
    }
    if (pass) {
      return "passAim";
    }
    return "idle";
  }

  private void tick(int loops) {
    for (int i = 0; i < loops; i++) {
      vision.periodic();
      mechanisms.updateInputs();
      interpreter.periodic(new InterpreterInputs(skill(), manualMode, manualFeed, intakeDeploy));
      SimHooks.stepTiming(0.02);
    }
  }

  private void settle() {
    interpreter.onEnable();
    tick(5);
    assertEquals(Phase.IDLE, interpreter.phase());
  }

  private double rps(String mech) {
    return mechanisms.commandedRps(mech);
  }

  @Test
  void idleTracksAndCommandsNothingHot() {
    settle();
    tick(25);
    assertEquals(Phase.IDLE, interpreter.phase());
    assertEquals(0.0, rps("shooter"));
  }

  @Test
  void intakeFlowReachesIntakingAndBack() {
    settle();
    intakeDeploy = true;
    tick(400);
    assertEquals(Phase.INTAKING, interpreter.phase());
    assertTrue(interpreter.intakeReflex().isOut());
    intakeDeploy = false;
    tick(400);
    assertEquals(Phase.IDLE, interpreter.phase());
  }

  @Test
  void fullScoreFlowIdleToShootingToClearingToIdle() {
    settle();
    fakeVisionIo.tagId = 18;
    shoot = true;
    tick(3);
    assertEquals(Phase.SPINNING_UP, interpreter.phase());
    tick(450);
    assertEquals(Phase.SHOOTING, interpreter.phase());
    // Gate open: all three stages feed (35/35/100).
    assertTrue(rps(IndexerFeed.HORIZONTAL) > 0.0);
    assertEquals(100.0, rps(IndexerFeed.UPWARD));
    shoot = false;
    tick(1);
    assertEquals(Phase.CLEARING, interpreter.phase());
    assertEquals(0.0, rps("shooter"));
    assertTrue(rps(IndexerFeed.UPWARD) < 0.0, "clearing reverses every stage full");
    tick(50);
    assertEquals(Phase.CLEARING, interpreter.phase()); // atomic: IDLE deferred
    tick(60);
    assertEquals(Phase.IDLE, interpreter.phase());
  }

  @Test
  void clearingPreemptionMatrix() {
    settle();
    fakeVisionIo.tagId = 18;
    shoot = true;
    tick(450);
    assertEquals(Phase.SHOOTING, interpreter.phase());
    shoot = false;
    tick(2);
    assertEquals(Phase.CLEARING, interpreter.phase());
    shoot = true; // re-press exits immediately (T23)
    tick(1);
    assertEquals(Phase.SPINNING_UP, interpreter.phase());

    tick(200);
    assertEquals(Phase.SHOOTING, interpreter.phase());
    shoot = false;
    tick(2);
    assertEquals(Phase.CLEARING, interpreter.phase());
    unjam = true; // UNJAM preempts the clear (T24)
    tick(1);
    assertEquals(Phase.UNJAMMING, interpreter.phase());
    unjam = false;
    tick(2);

    fakeVisionIo.tagId = 18;
    shoot = true;
    tick(400);
    assertEquals(Phase.SHOOTING, interpreter.phase());
    shoot = false;
    tick(2);
    assertEquals(Phase.CLEARING, interpreter.phase());
    manualMode = true; // manual preempts the clear (never blockable)
    tick(1);
    assertEquals(Phase.MANUAL, interpreter.phase());
    manualMode = false;
    tick(2);
    assertEquals(Phase.IDLE, interpreter.phase());
  }

  @Test
  void passFlowAimSpinFireAndClear() {
    settle();
    fakeVisionIo.tagId = 7;
    pass = true;
    tick(5);
    assertEquals(Phase.PASS_AIMING, interpreter.phase());
    assertEquals(0.0, rps("shooter"), "PASS_AIMING must not spin the shooter");
    shoot = true;
    tick(3);
    assertEquals(Phase.PASS_SPINNING_UP, interpreter.phase());
    assertEquals(95.0, rps("shooter"));
    tick(750);
    assertEquals(Phase.PASSING, interpreter.phase());
    shoot = false;
    tick(2);
    assertEquals(Phase.CLEARING, interpreter.phase());
    tick(110);
    assertEquals(Phase.PASS_AIMING, interpreter.phase());
    pass = false;
    tick(2);
    assertEquals(Phase.IDLE, interpreter.phase());
  }

  @Test
  void unjamFromIdleAndManualFromMidUnjam() {
    settle();
    unjam = true;
    tick(3);
    assertEquals(Phase.UNJAMMING, interpreter.phase());
    assertEquals(400.0, rps("shooter"), "W19: shooter forward at max during unjam");
    assertTrue(rps(IndexerFeed.VERTICAL) < 0.0, "W19: indexers reversed");
    manualMode = true;
    tick(2);
    assertEquals(Phase.MANUAL, interpreter.phase());
    assertEquals(0.0, rps("shooter"), "manual entry stops the shooter");
    manualMode = false;
    unjam = false;
    tick(2);
    assertEquals(Phase.IDLE, interpreter.phase());
  }

  @Test
  void onEnableResetsInterruptedStateToIdle() {
    settle();
    unjam = true;
    tick(3);
    assertEquals(Phase.UNJAMMING, interpreter.phase());
    unjam = false;
    interpreter.onEnable();
    tick(1);
    assertEquals(Phase.IDLE, interpreter.phase());
  }

  @Test
  void shootWhileIntakingReturnsToIntaking() {
    settle();
    fakeVisionIo.tagId = 18;
    intakeDeploy = true;
    tick(400);
    assertEquals(Phase.INTAKING, interpreter.phase());
    shoot = true;
    tick(450);
    assertEquals(Phase.SHOOTING, interpreter.phase());
    assertTrue(
        interpreter.intakeReflex().isOut(), "intake stays deployed through the shoot states");
    shoot = false;
    tick(2);
    assertEquals(Phase.CLEARING, interpreter.phase());
    tick(110);
    assertEquals(Phase.INTAKING, interpreter.phase());
    intakeDeploy = false;
    tick(400);
    assertEquals(Phase.IDLE, interpreter.phase());
  }

  @Test
  void shootWithoutVisionNeverFeeds() {
    // W12 end-to-end: no camera tag -> never aimed -> the feed gate never opens.
    settle();
    shoot = true; // no tag set
    tick(300);
    assertEquals(Phase.SPINNING_UP, interpreter.phase());
    assertFalse(interpreter.aiming().isAimed());
    assertEquals(0.0, rps(IndexerFeed.HORIZONTAL), "feed gate stays closed without aim");
    shoot = false;
    tick(5);
    assertEquals(Phase.IDLE, interpreter.phase());
  }
}
