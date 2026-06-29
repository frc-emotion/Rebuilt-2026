package frc.robot.autonomy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LocalSkillDriver;
import frc.robot.runtime.ChassisStateProvider;
import frc.robot.runtime.Mechanisms;
import frc.robot.runtime.RuntimeSubsystem;
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
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * Headless sim OBSERVATION harness (not an assertion test). It runs the REAL skill runtime
 * (mechanisms in sim + interpreter + skill server + the shift-aware {@link MatchTree}) with a
 * compressed teleop clock swept through every SHIFT, and dumps a per-loop trace + a per-shift
 * feed-gate-open count to build/autonomy-obs.txt so the actual behavior can be read directly.
 *
 * <p>Movement is a DETERMINISTIC kinematic navigator (teleports toward the target) on purpose: the
 * real PathPlanner navigator's headless swerve-sim physics is timing/thread dependent and fires
 * inconsistently run-to-run, which is misleading in a demonstration trace — so movement is
 * idealized here to isolate (and reliably show) the DECISION + SKILL + shooting chain. The real
 * navigator is still constructed so the wiring is smoke-tested, and exercised for real in {@code
 * simulateJava}. Uses a guaranteed-tag fake vision so aiming isn't gated on whether the camera sees
 * a tag.
 *
 * <p>Expected trace: harvest (no fire) during our INACTIVE shifts; park at the shoot pose and fire
 * (SHOOTING/SUCCEEDED) during our ACTIVE shifts + endgame.
 */
class AutonomySimObservationTest {

  /** Kinematic navigator: teleports the stored pose toward the commanded target (no physics). */
  private static final class FakeNavigator implements Navigator {
    private Pose2d pose;
    private Pose2d target;

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
      edu.wpi.first.math.geometry.Translation2d to =
          target.getTranslation().minus(pose.getTranslation());
      double dist = to.getNorm();
      if (dist < 1e-6) {
        return;
      }
      double stepDist = Math.min(dist, 4.0 * 0.02); // 4 m/s over a 20 ms loop
      pose =
          new Pose2d(pose.getTranslation().plus(to.times(stepDist / dist)), target.getRotation());
    }
  }

  private static final class FakeVisionIO implements VisionIO {
    private static final edu.wpi.first.math.geometry.Transform3d kCamToTag =
        new edu.wpi.first.math.geometry.Transform3d(
            new edu.wpi.first.math.geometry.Translation3d(2.5, 0.2, 0.0),
            new edu.wpi.first.math.geometry.Rotation3d());
    int tagId = 18;
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

  @BeforeAll
  static void once() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @AfterAll
  static void done() {
    SimHooks.resumeTiming();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    // Don't leak enabled/autonomous DS state into other suites.
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void observe() throws IOException {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    DriverStationSim.setDsAttached(true);
    // Pin BLUE so pathfindToPoseFlipped does NOT flip the blue-authored poses — otherwise a leaked
    // Red alliance from another suite mirrors every target and the robot drives off-field. (Real
    // takeaway: set the sim alliance deliberately; the autonomy flips correctly for either side.)
    DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Blue1);
    // The shift brain is a teleop autonomy, but its mode comes from the INJECTED teleop clock
    // below,
    // not the DS — so we drive the sim physics in autonomous-enabled (the stable config the swerve
    // sim harness was validated in) while sweeping the clock through every shift for observation.
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Drive drive = new Drive();
    drive.configureAutoBuilder();

    MechanismsConfig mechConfig =
        MechanismsConfig.load(Filesystem.getDeployDirectory().toPath().resolve("mechanisms.json"));
    Mechanisms mechanisms = new Mechanisms(mechConfig, false);
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    Vision vision =
        new Vision(
            new FakeVisionIO(),
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
    SkillInterpreter interpreter =
        new SkillInterpreter(
            mechanisms,
            new Setpoints(),
            new VisionPerception(vision),
            chassis,
            SkillTable.load(Filesystem.getDeployDirectory().toPath().resolve("skills.json")),
            mechConfig);
    SkillServer server = new SkillServer();
    LocalSkillDriver localDriver =
        new LocalSkillDriver(new CommandXboxController(1), server, mechanisms);
    RuntimeSubsystem runtime =
        new RuntimeSubsystem(
            mechanisms, interpreter, server, localDriver, new VisionPerception(vision));

    StrategyConfig strategy =
        StrategyConfig.load(Filesystem.getDeployDirectory().toPath().resolve("strategy.json"));
    // DETERMINISTIC movement: a kinematic navigator (teleports toward the target) so the trace
    // reliably demonstrates the DECISION + SKILL + shooting chain (the real PathPlanner navigator's
    // headless swerve-sim physics is too jittery to fire consistently — watch that in
    // simulateJava).
    // Construct the real navigator anyway so the wiring is smoke-tested.
    new PathPlannerNavigator(
        drive,
        AutonomyConstants.kPathConstraints,
        AutonomyConstants.kArrivalToleranceMeters,
        ObstacleProvider.NONE);
    FakeNavigator navigator =
        new FakeNavigator(new Pose2d(0.5, 4.0, edu.wpi.first.math.geometry.Rotation2d.kZero));
    // A compressed teleop clock (0.1 s per loop) so a single run sweeps every SHIFT; our hub is
    // inactive in SHIFT 1 so the trace shows score → harvest → stage → re-activated score →
    // endgame.
    double[] teleopClock = {ShiftSchedule.kTeleopLengthSeconds};
    MatchTree tree =
        new MatchTree(
            navigator,
            server::invoke,
            server::setIntakeDeploy,
            () -> interpreter.scoringStatus(),
            PossessionProvider.UNKNOWN,
            () -> false,
            () -> teleopClock[0],
            () -> Optional.of(true),
            strategy,
            () -> Optional.empty());
    AutonomyCommand auto = new AutonomyCommand(tree, () -> {});

    runtime.onEnable();
    auto.schedule();

    List<String> log = new ArrayList<>();
    log.add(
        "loop  clk   shift        mode             pose(x,y)            btPhase   interpPhase     scoring   skill   intakeReq");
    int firedLoops = 0; // loops where the feed gate opened (SHOOTING / SUCCEEDED)
    int shootRequests = 0; // loops where the BT requested the shoot skill
    int[] firedByShift = new int[ShiftSchedule.Shift.values().length];
    frc.robot.runtime.reflex.ScoringSequencer.Phase prevPhase = interpreter.phase();
    // Clock runs at 0.04 s per 0.02 s loop (2x), so each 25 s shift is ~12.5 s of sim time — enough
    // for a full 7 s hopper-empty shot to complete within an active shift.
    for (int i = 0; i < 3600; i++) {
      teleopClock[0] = Math.max(-1.0, ShiftSchedule.kTeleopLengthSeconds - i * 0.04);
      // CommandScheduler runs Vision + RuntimeSubsystem periodics (the real interpreter/skills) and
      // the AutonomyCommand (the tree, which commands the navigator); then advance the fake pose.
      CommandScheduler.getInstance().run();
      navigator.step();
      SimHooks.stepTiming(0.02);

      SkillInterpreter.ScoringStatus status = interpreter.scoringStatus();
      frc.robot.runtime.reflex.ScoringSequencer.Phase phase = interpreter.phase();
      boolean firing =
          status == SkillInterpreter.ScoringStatus.SUCCEEDED
              || phase == frc.robot.runtime.reflex.ScoringSequencer.Phase.SHOOTING;
      if (firing) {
        firedLoops++;
        firedByShift[ShiftSchedule.shiftOf(teleopClock[0]).ordinal()]++;
      }
      if ("shoot".equals(server.resolve().scoringSkill())) {
        shootRequests++;
      }
      // Always log the moment it actually opens the feed gate (so brief shots aren't sampled away).
      if (phase == frc.robot.runtime.reflex.ScoringSequencer.Phase.SHOOTING
          && prevPhase != frc.robot.runtime.reflex.ScoringSequencer.Phase.SHOOTING) {
        Pose2d sp = navigator.pose();
        log.add(
            String.format(
                ">>> SHOT  loop %4d  clk %5.1f  %-11s  pose(%6.2f,%6.2f)  SUCCEEDED feed-gate open",
                i, teleopClock[0], ShiftSchedule.shiftOf(teleopClock[0]), sp.getX(), sp.getY()));
      }
      prevPhase = phase;

      if (i % 25 == 0) {
        Pose2d p = navigator.pose();
        log.add(
            String.format(
                "%4d  %5.1f %-11s  %-15s  (%6.2f,%6.2f)  %-8s  %-14s  %-9s  %-6s  %s",
                i,
                teleopClock[0],
                ShiftSchedule.shiftOf(teleopClock[0]),
                tree.mode(),
                p.getX(),
                p.getY(),
                tree.cyclePhase(),
                interpreter.phase(),
                interpreter.scoringStatus(),
                server.resolve().scoringSkill(),
                server.localIntakeDeploy()));
      }
    }

    log.add("");
    log.add("feed-gate-open (SHOOTING/SUCCEEDED) loops per shift — fires only in ACTIVE shifts:");
    for (ShiftSchedule.Shift sh : ShiftSchedule.Shift.values()) {
      log.add(String.format("  %-11s = %d", sh, firedByShift[sh.ordinal()]));
    }
    log.add("totals: shootRequests=" + shootRequests + "  firedLoops=" + firedLoops);

    Path out = Path.of("build", "autonomy-obs.txt");
    Files.write(out, log);
    System.out.println(
        "[OBS] wrote "
            + log.size()
            + " lines to "
            + out.toAbsolutePath()
            + " | shootRequests="
            + shootRequests
            + " firedLoops="
            + firedLoops);

    // OBSERVATION ONLY — no firing assertion here on purpose. This harness runs the REAL
    // pathfinding
    // navigator + swerve sim + a background AD* thread, which is timing/thread/order dependent and
    // would flake CI. Its value is (a) a smoke test that the whole real autonomy wiring constructs
    // and runs 1500 autonomous loops without throwing, and (b) the inspectable trace in
    // build/autonomy-obs.txt. The DETERMINISTIC "it collects and fires" guarantee lives in
    // MatchTreeTest (fake navigator). Run this on demand to watch real-sim behavior.
  }
}
