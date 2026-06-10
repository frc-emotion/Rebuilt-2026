package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodIOReal;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIO.Stage;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionPoseEstimator;
import frc.robot.superstructure.Goal;
import frc.robot.superstructure.Superstructure;
import java.io.IOException;
import java.nio.file.Path;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/** Wiring only: construct subsystems, bind goals to buttons, build autos. No logic. */
@Logged
public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(RobotConstants.kDriverPort);
  private final CommandXboxController operator =
      new CommandXboxController(RobotConstants.kOperatorPort);

  private final Drive drive = new Drive();
  private final AprilTagFieldLayout fieldLayout = loadFieldLayout();

  public final Turret turret;
  public final Hood hood;
  public final Shooter shooter;
  public final Indexer indexer;
  public final Intake intake;
  public final Vision vision;
  public final Superstructure superstructure;

  private final Telemetry telemetry = new Telemetry();
  private final edu.wpi.first.wpilibj.smartdashboard.SendableChooser<Command> autoChooser;

  public RobotContainer() {
    boolean real = RobotBase.isReal();

    turret =
        RobotConstants.kEnableTurret
            ? new Turret(real ? new TurretIOReal(RobotConstants.kMechanismBus) : new TurretIOSim())
            : null;
    hood =
        RobotConstants.kEnableHood
            ? new Hood(real ? new HoodIOReal(RobotConstants.kMechanismBus) : new HoodIOSim())
            : null;
    shooter =
        RobotConstants.kEnableShooter
            ? new Shooter(real ? new ShooterIOReal(RobotConstants.kMechanismBus) : new ShooterIOSim())
            : null;
    indexer =
        RobotConstants.kEnableIndexer
            ? new Indexer(real ? new IndexerIOReal(RobotConstants.kMechanismBus) : new IndexerIOSim())
            : null;
    intake =
        RobotConstants.kEnableIntake
            ? new Intake(real ? new IntakeIOReal(RobotConstants.kMechanismBus) : new IntakeIOSim())
            : null;

    VisionPoseEstimator poseEstimator =
        new VisionPoseEstimator(fieldLayout, drive::addVisionMeasurement);
    vision =
        RobotConstants.kEnableVision
            ? new Vision(
                real
                    ? new VisionIOReal()
                    : new VisionIOSim(
                        fieldLayout,
                        drive::getPose,
                        () -> turret != null ? turret.getPositionRot() : 0.0),
                poseEstimator,
                () -> turret != null ? turret.getPositionRot() : 0.0,
                () -> turret != null ? turret.getVelocityRps() : 0.0)
            : null;

    boolean allPresent =
        turret != null && hood != null && shooter != null && indexer != null && intake != null
            && vision != null;
    superstructure =
        allPresent
            ? new Superstructure(
                drive,
                turret,
                hood,
                shooter,
                indexer,
                intake,
                vision,
                () -> operator.getRightTriggerAxis() > 0.5,
                operator.leftBumper(),
                operator.rightStick(),
                () -> operator.getLeftTriggerAxis() > 0.5)
            : null;
    if (!allPresent) {
      DriverStation.reportWarning(
          "Superstructure disabled: one or more mechanisms are feature-flagged off", false);
    }

    DriverStation.silenceJoystickConnectionWarning(true);

    configureDriverBindings();
    configureOperatorBindings();

    drive.configureAutoBuilder();
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
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

    // Uncomment for interp-table calibration sessions only (requires manual mode active so the
    // Superstructure does not fight the command):
    // driver.b().whileTrue(new frc.robot.commands.CalibrationCommand(turret, hood, shooter,
    //     indexer, vision));
  }

  private void configureOperatorBindings() {
    if (superstructure == null) {
      return;
    }
    Trigger manual = new Trigger(superstructure::isManualMode);

    // Goal inputs (RT/LB/right-stick/LT) flow into the Superstructure as suppliers — see ctor.
    operator.a().onTrue(Commands.runOnce(superstructure::toggleIntake));
    operator.start().onTrue(Commands.runOnce(superstructure::toggleManualMode));
    operator
        .rightBumper()
        .onTrue(turret.runOnce(() -> {
          turret.zeroAtCurrentPosition();
          System.out.println("[TURRET] Zeroed at current position");
        }));

    // ── Manual mode (design §7): operator drives mechanisms directly; the Superstructure
    // commands nothing. Default commands are no-ops outside manual so they never fight it. ──
    turret.setDefaultCommand(
        turret.run(() -> {
          if (superstructure.isManualMode()) {
            turret.setManualVoltage(
                MathUtil.applyDeadband(
                    operator.getRightX(), Superstructure.kManualJoystickDeadband));
          }
        }));
    hood.setDefaultCommand(
        hood.run(() -> {
          if (superstructure.isManualMode()) {
            hood.setVoltage(-operator.getRightY() * HoodConstants.kManualVolts);
          }
        }));

    manual
        .and(operator.rightTrigger())
        .whileTrue(
            Commands.startEnd(
                () -> shooter.setVelocity(RotationsPerSecond.of(ShooterConstants.kManualRps)),
                shooter::stop,
                shooter));
    manual
        .and(operator.leftTrigger())
        .whileTrue(
            Commands.startEnd(
                () -> indexer.setVelocity(Stage.VERTICAL, IndexerConstants.kVerticalSpeedRps),
                () -> indexer.stop(Stage.VERTICAL),
                indexer));
    manual
        .and(operator.rightStick())
        .whileTrue(
            Commands.runEnd(
                () -> {
                  indexer.setVelocity(
                      Stage.HORIZONTAL,
                      -IndexerConstants.kHorizontalSpeedRps * IndexerConstants.kUnjamFraction);
                  indexer.setVelocity(
                      Stage.VERTICAL,
                      -IndexerConstants.kVerticalSpeedRps * IndexerConstants.kUnjamFraction);
                  indexer.setVelocity(
                      Stage.UPWARD,
                      -IndexerConstants.kUpwardSpeedRps * IndexerConstants.kUnjamFraction);
                  shooter.setVelocity(RotationsPerSecond.of(ShooterConstants.kMaxRps));
                },
                () -> {
                  shooter.stop();
                  indexer.stopAll();
                },
                indexer,
                shooter));

    // Turret POV presets and hood X/Y/B presets — direct setpoints, manual mode only.
    manual.and(operator.povUp()).whileTrue(turretPreset(TurretConstants.kPresetForwardRot));
    manual.and(operator.povDown()).whileTrue(turretPreset(TurretConstants.kPresetBackRot));
    manual.and(operator.povLeft()).whileTrue(turretPreset(TurretConstants.kPresetLeftRot));
    manual.and(operator.povRight()).whileTrue(turretPreset(TurretConstants.kPresetRightRot));
    manual.and(operator.x()).whileTrue(hoodPreset(HoodConstants.kPresetDownRot));
    manual.and(operator.y()).whileTrue(hoodPreset(HoodConstants.kPresetMidRot));
    manual.and(operator.b()).whileTrue(hoodPreset(HoodConstants.kPresetUpRot));
  }

  private Command turretPreset(double rot) {
    return turret.run(() -> turret.setTargetPosition(Rotations.of(rot)));
  }

  private Command hoodPreset(double rot) {
    return hood.run(() -> hood.setAngle(Rotations.of(rot)));
  }

  // Exact legacy strings — the deploy .auto files reference these by name.
  private void registerNamedCommands() {
    if (superstructure == null) {
      return;
    }
    NamedCommands.registerCommand(
        "intakeOut", Commands.runOnce(() -> superstructure.setIntakeRequested(true)));
    NamedCommands.registerCommand(
        "intakeIn", Commands.runOnce(() -> superstructure.setIntakeRequested(false)));
    NamedCommands.registerCommand("shoot", superstructure.goalCommand(Goal.SHOOT));
    // autoShoot (deprecated AutoShootCommand) and feedIndexers both become a held SHOOT goal —
    // the feed gate replaces the old pre-spin/ungated-feed split (design D12, step 8).
    NamedCommands.registerCommand("autoShoot", superstructure.goalCommand(Goal.SHOOT));
    NamedCommands.registerCommand("feedIndexers", superstructure.goalCommand(Goal.SHOOT));
    NamedCommands.registerCommand(
        "stopAll", Commands.runOnce(superstructure::stopAllMechanisms));
    NamedCommands.registerCommand("reverseIndexer", superstructure.goalCommand(Goal.UNJAM));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }
}
