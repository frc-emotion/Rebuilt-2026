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

    // v1 match autonomy: a behavior tree that plays the whole match on its own (collect↔shoot)
    // using
    // the existing skills + PathPlanner pathfinding. Selectable from the auto chooser. See
    // docs/autonomy.md. Opponent/possession/learned-nav-cost are empty seams (no sensors yet).
    PathPlannerNavigator navigator =
        new PathPlannerNavigator(
            drive, AutonomyConstants.kPathConstraints, AutonomyConstants.kArrivalToleranceMeters);
    MatchTree matchTree =
        new MatchTree(
            navigator,
            skillServer::invoke,
            skillServer::setIntakeDeploy,
            () -> runtime.interpreter().scoringStatus(),
            PossessionProvider.UNKNOWN,
            () -> false, // human-takeover gate stub (no takeover signal yet)
            DriverStation::getMatchTime);
    autoChooser.addOption("BT Match (autonomy)", new AutonomyCommand(matchTree));

    SmartDashboard.putData("Auto Chooser", autoChooser);

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
