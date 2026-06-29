package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autonomy.AutonomyCommand;
import frc.robot.autonomy.AutonomyConstants;
import frc.robot.autonomy.MatchTree;
import frc.robot.autonomy.PathPlannerNavigator;
import frc.robot.autonomy.PossessionProvider;
import frc.robot.autonomy.ShiftSchedule;
import frc.robot.autonomy.SimOpponentProvider;
import frc.robot.autonomy.StrategyConfig;
import frc.robot.runtime.ChassisStateProvider;
import frc.robot.runtime.Mechanisms;
import frc.robot.runtime.RuntimeSubsystem;
import frc.robot.runtime.Setpoints;
import frc.robot.runtime.SkillInterpreter;
import frc.robot.runtime.SkillServer;
import frc.robot.runtime.config.MechanismsConfig;
import frc.robot.runtime.config.SkillTable;
import frc.robot.runtime.perception.NoPerception;
import frc.robot.runtime.perception.PerceptionProvider;
import frc.robot.runtime.perception.VisionPerception;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionPoseEstimator;
import java.io.IOException;
import java.nio.file.Path;

/**
 * Wiring only (skill-server architecture): construct the mechanism layer from {@code
 * mechanisms.json}, the skill interpreter from {@code skills.json}, the reflexes, the perception
 * provider (local Vision behind the seam), the NT skill server, and the local operator driver.
 * Drive stays the unmodified CTRE adapter. No robot logic lives here.
 */
@Logged
public class RobotContainer {
  private final CommandXboxController driver =
      new CommandXboxController(RobotConstants.kDriverPort);
  private final CommandXboxController operator =
      new CommandXboxController(RobotConstants.kOperatorPort);

  private final Drive drive = new Drive();
  private final AprilTagFieldLayout fieldLayout = loadFieldLayout();

  public final Mechanisms mechanisms;
  public final Vision vision; // null when vision is feature-flagged off
  public final RuntimeSubsystem runtime;
  private final SkillServer skillServer;
  private final LocalSkillDriver localDriver;

  private final Telemetry telemetry = new Telemetry();
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    boolean real = RobotBase.isReal();

    MechanismsConfig mechConfig =
        MechanismsConfig.load(Filesystem.getDeployDirectory().toPath().resolve("mechanisms.json"));
    SkillTable skillTable =
        SkillTable.load(Filesystem.getDeployDirectory().toPath().resolve("skills.json"));
    mechanisms = new Mechanisms(mechConfig, real);

    // Vision (constructed BEFORE the runtime subsystem so its periodic runs first each loop).
    VisionPoseEstimator poseEstimator =
        new VisionPoseEstimator(fieldLayout, drive::addVisionMeasurement);
    PerceptionProvider perception;
    if (RobotConstants.kEnableVision) {
      vision =
          new Vision(
              real
                  ? new VisionIOReal()
                  : new VisionIOSim(
                      fieldLayout, drive::getPose, () -> mechanisms.read("turret").positionRot()),
              poseEstimator,
              () -> mechanisms.read("turret").positionRot(),
              () -> mechanisms.read("turret").velocityRps());
      perception = new VisionPerception(vision);
    } else {
      vision = null;
      perception = new NoPerception();
      DriverStation.reportWarning("Vision feature-flagged off — running with NoPerception.", false);
    }

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
            mechanisms, new Setpoints(), perception, chassis, skillTable, mechConfig);
    skillServer = new SkillServer();
    localDriver = new LocalSkillDriver(operator, skillServer, mechanisms);
    runtime = new RuntimeSubsystem(mechanisms, interpreter, skillServer, localDriver, perception);

    DriverStation.silenceJoystickConnectionWarning(true);

    configureDriverBindings();
    localDriver.configureBindings();

    drive.configureAutoBuilder();
    localDriver.registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();

    // Shift-aware teleop autonomy: a behavior tree that plays the whole TELEOP on its own, driven
    // by
    // the editable strategy.json + the 2026 SHIFT clock (see docs/strategy.md). AUTO stays on the
    // PathPlanner routine picked above; this brain takes over teleop ONLY when the dashboard toggle
    // "TeleopAutonomy" is on, so manual driving is always available (non-negotiable #5). Targets
    // are
    // clamped legal by LegalRegion; opponent/possession/ball-detection are empty seams (no sensors
    // yet).
    StrategyConfig strategy =
        StrategyConfig.load(Filesystem.getDeployDirectory().toPath().resolve("strategy.json"));

    // Sim/debug mode override: a plain editable string next to TeleopAutonomy so you can FORCE a
    // mode
    // without running a practice match + game-data string. Valid values: MATCH (default, real shift
    // clock), ACTIVE, INACTIVE, ENDGAME (see ShiftSchedule.parseModeOverride).
    SmartDashboard.putString("AutonomyModeOverride", "MATCH");

    // Dynamic obstacle avoidance: a sim opponent you can place/move from the dashboard (the seam
    // the
    // real robot detector will replace). The navigator pushes it into PathPlanner each loop.
    PathPlannerNavigator navigator =
        new PathPlannerNavigator(
            drive,
            AutonomyConstants.kPathConstraints,
            AutonomyConstants.kArrivalToleranceMeters,
            new SimOpponentProvider());
    MatchTree matchTree =
        new MatchTree(
            navigator,
            skillServer::invoke,
            skillServer::setIntakeDeploy,
            () -> runtime.interpreter().scoringStatus(),
            PossessionProvider.UNKNOWN,
            () -> false, // human-takeover gate stub (no takeover signal yet)
            // Teleop clock remaining (s); <0 outside teleop so the shift schedule reads PRE_MATCH.
            () -> DriverStation.isTeleopEnabled() ? DriverStation.getMatchTime() : -1.0,
            // FMS auto-winner game data → which hub is inactive in SHIFT 1 (empty => assume
            // active).
            () ->
                ShiftSchedule.parseOurHubInactiveFirst(
                    DriverStation.getGameSpecificMessage(),
                    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)),
            strategy,
            () ->
                ShiftSchedule.parseModeOverride(
                    SmartDashboard.getString("AutonomyModeOverride", "MATCH")));

    // In SIM the robot boots at the (0,0) corner — a blocked navgrid cell, so pathfinding can't
    // start and the robot never moves (and AD* thrashes, causing loop overruns). Seed a legal
    // start.
    Runnable seedSimPose =
        () -> {
          if (RobotBase.isSimulation() && drive.getPose().getTranslation().getNorm() < 0.5) {
            drive.resetPose(AutonomyConstants.kSimStartPose);
          }
        };

    // EASIEST sim path: run the shift brain straight from the auto chooser (one Autonomous+Enable,
    // no teleop toggle dance). Use AutonomyModeOverride to force a mode while watching.
    autoChooser.addOption("Shift Brain (autonomy)", new AutonomyCommand(matchTree, seedSimPose));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Real-match path: take over TELEOP when the dashboard toggle is on (manual driving otherwise).
    SmartDashboard.putBoolean("TeleopAutonomy", false);
    RobotModeTriggers.teleop()
        .and(() -> SmartDashboard.getBoolean("TeleopAutonomy", false))
        .whileTrue(new AutonomyCommand(matchTree, seedSimPose));

    drive.registerTelemetry(telemetry::telemeterize);
  }

  private static AprilTagFieldLayout loadFieldLayout() {
    Path deployed = Filesystem.getDeployDirectory().toPath().resolve("FRC2026_WELDED.json");
    try {
      return new AprilTagFieldLayout(deployed);
    } catch (IOException e) {
      DriverStation.reportError(
          "Could not load deployed field layout, falling back to bundled default: " + e, false);
      return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
  }

  private void configureDriverBindings() {
    drive
        .subsystem()
        .setDefaultCommand(
            drive.teleopDriveCommand(driver::getLeftY, driver::getLeftX, driver::getRightX));
    driver.a().whileTrue(drive.brakeCommand());
    driver.leftBumper().onTrue(drive.seedFieldCentricCommand());
    RobotModeTriggers.disabled().whileTrue(drive.idleWhileDisabledCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public RuntimeSubsystem getRuntime() {
    return runtime;
  }
}
