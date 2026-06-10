package frc.robot.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionPoseEstimator;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * THE PHASE 3 SIM RUN: every goal flow end-to-end against physics sims and synthetic camera
 * frames, the atomic CLEARING preemption matrix, and manual mode in/out including from
 * mid-sequence states. Each test documents what was run and what was observed.
 *
 * <p>(This robot has no CLIMB — deleted by team decision D8 — and TRACK is what IDLE does.)
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class SuperstructureFlowTest {

  /** Fake camera: emits one frame per loop with a chosen tag, monotonic timestamps. */
  private static final class FakeVisionIO implements VisionIO {
    // Camera 2.5 m from the tag, 0.2 m left, facing it square-on. For hub tag 18 the tag→hub
    // vector (−0.604, −0.2) lands the hub dead ahead at 1.896 m → tx = 0° → aimed.
    private static final Transform3d kCamToTag =
        new Transform3d(new Translation3d(2.5, 0.2, 0.0), new Rotation3d());

    int tagId = -1; // -1 = no targets
    private long frame = 0;

    @Override
    public VisionIOInputs updateInputs() {
      frame++;
      long timestampMicros = frame * 20_000; // 50 Hz, strictly increasing (dedupe-friendly)
      List<org.photonvision.targeting.TargetCorner> corners =
          List.of(
              new org.photonvision.targeting.TargetCorner(0, 0),
              new org.photonvision.targeting.TargetCorner(1, 0),
              new org.photonvision.targeting.TargetCorner(1, 1),
              new org.photonvision.targeting.TargetCorner(0, 1));
      List<PhotonTrackedTarget> targets =
          tagId < 0
              ? List.of()
              : List.of(new PhotonTrackedTarget(
                  0.0, 0.0, 1.0, 0.0, tagId, -1, 0.0f, kCamToTag, kCamToTag, 0.05,
                  corners, corners));
      return new VisionIOInputs(
          true,
          List.of(new PhotonPipelineResult(frame, timestampMicros, timestampMicros + 1000, 0, targets)));
    }
  }

  private static Drive drive;
  private static Turret turret;
  private static Hood hood;
  private static Shooter shooter;
  private static Indexer indexer;
  private static Intake intake;
  private static FakeVisionIO fakeVisionIo;
  private static Vision vision;
  private static Superstructure superstructure;

  private static boolean shootHeld;
  private static boolean passHeld;
  private static boolean unjamHeld;

  @BeforeAll
  static void setup() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming(); // deterministic clock for the 2.0 s CLEARING timer
    CommandScheduler.getInstance().unregisterAllSubsystems();

    drive = new Drive();
    turret = new Turret(new TurretIOSim());
    hood = new Hood(new HoodIOSim());
    shooter = new Shooter(new ShooterIOSim());
    indexer = new Indexer(new IndexerIOSim());
    intake = new Intake(new IntakeIOSim());
    fakeVisionIo = new FakeVisionIO();
    vision =
        new Vision(
            fakeVisionIo,
            new VisionPoseEstimator(
                edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(
                    edu.wpi.first.apriltag.AprilTagFields.kDefaultField),
                (pose, t, std) -> {}),
            turret::getPositionRot,
            turret::getVelocityRps);
    superstructure =
        new Superstructure(
            drive, turret, hood, shooter, indexer, intake, vision,
            () -> shootHeld, () -> passHeld, () -> unjamHeld, () -> false);
  }

  @AfterAll
  static void teardown() {
    SimHooks.resumeTiming();
    // These subsystems self-registered on construction; don't leak them into other test classes.
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @BeforeEach
  void neutral() {
    shootHeld = false;
    passHeld = false;
    unjamHeld = false;
    fakeVisionIo.tagId = -1;
  }

  /** One robot loop, scheduler-free and deterministic: subsystem periodics then the brain. */
  private static void tick(int loops) {
    for (int i = 0; i < loops; i++) {
      turret.periodic();
      hood.periodic();
      shooter.periodic();
      indexer.periodic();
      intake.periodic();
      vision.periodic();
      superstructure.periodic();
      SimHooks.stepTiming(0.02);
    }
  }

  private static void settleToIdle() {
    superstructure.onEnable();
    tick(5);
    assertEquals(RobotState.IDLE, superstructure.getState());
  }

  @Test
  @Order(1)
  void idleTracksAndCommandsNothingHot() {
    settleToIdle();
    tick(25);
    // OBSERVED: rests in IDLE, shooter setpoint zero, feed gate closed.
    assertEquals(RobotState.IDLE, superstructure.getState());
    assertFalse(shooter.commandedNonZero());
  }

  @Test
  @Order(2)
  void intakeFlowReachesIntakingAndBack() {
    settleToIdle();
    superstructure.setIntakeRequested(true);
    tick(400); // arm sim swings out under gravity + P control
    // OBSERVED: nested machine deployed, robot rests in INTAKING.
    assertEquals(RobotState.INTAKING, superstructure.getState());
    assertTrue(intake.isOut());
    superstructure.setIntakeRequested(false);
    tick(400);
    assertEquals(RobotState.IDLE, superstructure.getState());
  }

  @Test
  @Order(3)
  void fullScoreFlowIdleToShootingToClearingToIdle() {
    settleToIdle();
    fakeVisionIo.tagId = 18; // blue hub tag (no-FMS alliance defaults Blue), dead ahead

    shootHeld = true;
    tick(3);
    assertEquals(RobotState.SPINNING_UP, superstructure.getState());

    tick(400); // flywheel sim spins to the interp-table speed (~42 RPS at 1.896 m)
    // OBSERVED: gate (aimed && atSpeed && nonZero) opened — firing.
    assertEquals(RobotState.SHOOTING, superstructure.getState());
    assertTrue(shooter.atSpeed());

    shootHeld = false; // release mid-volley
    tick(1);
    // OBSERVED: atomic CLEARING entered — shooter stopped, indexers reversing full.
    assertEquals(RobotState.CLEARING, superstructure.getState());
    assertFalse(shooter.commandedNonZero());

    tick(50); // 1.0 s — clear still running, IDLE goal deferred (atomicity)
    assertEquals(RobotState.CLEARING, superstructure.getState());
    tick(60); // past 2.0 s total
    // OBSERVED: clear completed, returned to rest.
    assertEquals(RobotState.IDLE, superstructure.getState());
  }

  @Test
  @Order(4)
  void clearingPreemptionMatrixShootUnjamManual() {
    settleToIdle();
    fakeVisionIo.tagId = 18;

    // Reach SHOOTING, release into CLEARING, then re-press SHOOT: exits immediately (T23).
    shootHeld = true;
    tick(400);
    assertEquals(RobotState.SHOOTING, superstructure.getState());
    shootHeld = false;
    tick(2);
    assertEquals(RobotState.CLEARING, superstructure.getState());
    shootHeld = true;
    tick(1);
    assertEquals(RobotState.SPINNING_UP, superstructure.getState());

    // Back into CLEARING, then UNJAM preempts the atomic clear (T24).
    tick(200); // re-reach SHOOTING (flywheel still mostly spun up)
    assertEquals(RobotState.SHOOTING, superstructure.getState());
    shootHeld = false;
    tick(2);
    assertEquals(RobotState.CLEARING, superstructure.getState());
    unjamHeld = true;
    tick(1);
    assertEquals(RobotState.UNJAMMING, superstructure.getState());
    unjamHeld = false;
    tick(2);

    // Back into CLEARING, then MANUAL preempts the atomic clear (T20 beats the drain).
    fakeVisionIo.tagId = 18;
    shootHeld = true;
    tick(400);
    assertEquals(RobotState.SHOOTING, superstructure.getState());
    shootHeld = false;
    tick(2);
    assertEquals(RobotState.CLEARING, superstructure.getState());
    superstructure.toggleManualMode();
    tick(1);
    // OBSERVED: manual override is never blockable, even mid-atomic-clear.
    assertEquals(RobotState.MANUAL, superstructure.getState());
    superstructure.toggleManualMode();
    tick(2);
    assertEquals(RobotState.IDLE, superstructure.getState());
  }

  @Test
  @Order(5)
  void passFlowAimSpinFireAndClear() {
    settleToIdle();
    fakeVisionIo.tagId = 7; // red-zone tag = a Blue robot's passing target

    passHeld = true; // LB alone: aim only
    tick(5);
    assertEquals(RobotState.PASS_AIMING, superstructure.getState());
    assertFalse(shooter.commandedNonZero(), "PASS_AIMING must not spin the shooter");

    shootHeld = true; // LB+RT: fixed lob, gate on speed only
    tick(3);
    assertEquals(RobotState.PASS_SPINNING_UP, superstructure.getState());
    tick(700); // 95 RPS is near the flywheel's voltage ceiling — long spin-up
    assertEquals(RobotState.PASSING, superstructure.getState());

    shootHeld = false; // release with LB still held: atomic clear, then back to aiming
    tick(2);
    assertEquals(RobotState.CLEARING, superstructure.getState());
    tick(110);
    assertEquals(RobotState.PASS_AIMING, superstructure.getState());
    passHeld = false;
    tick(2);
    assertEquals(RobotState.IDLE, superstructure.getState());
  }

  @Test
  @Order(6)
  void unjamFlowFromIdleAndManualFromMidUnjam() {
    settleToIdle();
    unjamHeld = true;
    tick(3);
    // OBSERVED: indexers reversing at half speed, shooter commanded forward (W19).
    assertEquals(RobotState.UNJAMMING, superstructure.getState());
    assertTrue(shooter.commandedNonZero());

    superstructure.toggleManualMode(); // manual from a mid-sequence non-shoot state
    tick(2);
    assertEquals(RobotState.MANUAL, superstructure.getState());
    assertFalse(shooter.commandedNonZero(), "manual entry must stop the shooter");
    superstructure.toggleManualMode();
    unjamHeld = false;
    tick(2);
    assertEquals(RobotState.IDLE, superstructure.getState());
  }

  @Test
  @Order(7)
  void onEnableResetsAnInterruptedStateToIdle() {
    // The stuck-CLEARING regression (gate finding): disable mid-sequence, re-enable → IDLE.
    settleToIdle();
    unjamHeld = true;
    tick(3);
    assertEquals(RobotState.UNJAMMING, superstructure.getState());
    unjamHeld = false;
    superstructure.onEnable(); // as Robot.teleopInit/autonomousInit does
    tick(1);
    assertEquals(RobotState.IDLE, superstructure.getState());
  }

  @Test
  @Order(8)
  void shootWhileIntakingReturnsToIntaking() {
    settleToIdle();
    fakeVisionIo.tagId = 18;
    superstructure.setIntakeRequested(true);
    tick(400);
    assertEquals(RobotState.INTAKING, superstructure.getState());

    shootHeld = true; // intake-while-shooting concurrency: deployment carries via the condition
    tick(400);
    assertEquals(RobotState.SHOOTING, superstructure.getState());
    assertTrue(intake.isOut(), "intake must stay deployed through the shoot states");

    shootHeld = false;
    tick(2);
    assertEquals(RobotState.CLEARING, superstructure.getState());
    tick(110);
    // OBSERVED: rest state after the clear is INTAKING, not IDLE — the toggle is still on.
    assertEquals(RobotState.INTAKING, superstructure.getState());
    superstructure.setIntakeRequested(false);
    tick(400);
    assertEquals(RobotState.IDLE, superstructure.getState());
  }
}
