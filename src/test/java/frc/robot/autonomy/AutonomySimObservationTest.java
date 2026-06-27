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
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * Headless sim OBSERVATION harness (not an assertion test). It wires the REAL autonomy exactly as
 * the robot does — real Drive + the real PathPlannerNavigator (actual pathfinding) + the swerve sim
 * advanced manually via {@code updateSimState} + DriverStationSim set to autonomous+enabled — and
 * dumps a per-loop trace (pose, BT phase, interpreter phase + scoring status, resolved skill) to
 * build/autonomy-obs.txt so the actual sim behavior can be read directly. Uses a guaranteed-tag
 * fake vision to ISOLATE the decision/movement path from whether the camera happens to see a tag.
 */
class AutonomySimObservationTest {

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

    PathPlannerNavigator navigator =
        new PathPlannerNavigator(
            drive, AutonomyConstants.kPathConstraints, AutonomyConstants.kArrivalToleranceMeters);
    MatchTree tree =
        new MatchTree(
            navigator,
            server::invoke,
            server::setIntakeDeploy,
            () -> interpreter.scoringStatus(),
            PossessionProvider.UNKNOWN,
            () -> false,
            () -> -1.0);
    AutonomyCommand auto = new AutonomyCommand(tree);

    runtime.onEnable();
    auto.schedule();

    List<String> log = new ArrayList<>();
    log.add(
        "loop  pose(x,y)            btPhase   interpPhase     scoring   resolvedSkill  intakeReq  pathfindCfg");
    int firedLoops = 0; // loops where the feed gate opened (SHOOTING / SUCCEEDED)
    int shootRequests = 0; // loops where the BT requested the shoot skill
    for (int i = 0; i < 1500; i++) {
      CommandScheduler.getInstance().run(); // subsystem periodics + the autonomy command
      for (int s = 0; s < 5; s++) {
        drive.subsystem().updateSimState(0.004, 12.0); // match the real 4 ms swerve sim cadence
      }
      SimHooks.stepTiming(0.02);

      SkillInterpreter.ScoringStatus status = interpreter.scoringStatus();
      if (status == SkillInterpreter.ScoringStatus.SUCCEEDED
          || interpreter.phase() == frc.robot.runtime.reflex.ScoringSequencer.Phase.SHOOTING) {
        firedLoops++;
      }
      if ("shoot".equals(server.resolve().scoringSkill())) {
        shootRequests++;
      }

      if (i % 25 == 0) {
        Pose2d p = drive.getPose();
        log.add(
            String.format(
                "%4d  (%6.2f,%6.2f)  %-8s  %-14s  %-8s  %-12s  %-8s  %s",
                i,
                p.getX(),
                p.getY(),
                tree.cyclePhase(),
                interpreter.phase(),
                interpreter.scoringStatus(),
                server.resolve().scoringSkill(),
                server.localIntakeDeploy(),
                com.pathplanner.lib.auto.AutoBuilder.isPathfindingConfigured()));
      }
    }

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
