package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.CANID;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.CalibrationShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurretAutoAimCommand;
import frc.robot.commands.indexer.indexerDefault;
import frc.robot.commands.indexer.reverseIndexers;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

import frc.robot.Constants.Gen;

@Logged
public class RobotContainer {

        //  DRIVE

        private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        //  CONTROLLERS

        private final CommandXboxController joystick = new CommandXboxController(Gen.driverPort);
        public static CommandXboxController operator = new CommandXboxController(Gen.operatorPort);
        
        //  SUBSYSTEMS

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Vision vision = (Gen.enableVision) ? new Vision() : null;
        public final Intake intake = (Gen.enableIntake) ? new Intake(Gen.mechanismBus) : null;
        public final Indexer indexer = (Gen.enableIndexer) ? new Indexer(Gen.mechanismBus) : null;
        public final Turret turret = (Gen.enableTurret) ? new Turret(Gen.mechanismBus) : null;
        public final Hood hood = (Gen.enableHood) ? new Hood(Gen.mechanismBus) : null;
        public final Shooter shooter = (Gen.enableShooter) ? new Shooter(Gen.mechanismBus) : null;

        public static TurretAutoAimCommand visionAutoAim;

        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                edu.wpi.first.wpilibj.DriverStation.silenceJoystickConnectionWarning(true);

                if (turret != null){

                        visionAutoAim = new TurretAutoAimCommand(drivetrain, vision, turret,
                                () -> operator.getRightX(),
                                () -> operator.leftStick().getAsBoolean(),
                                () -> operator.leftBumper().getAsBoolean());
                        }
                else{
                        visionAutoAim = null;
                }

                if (hood != null){
                        hood.setDefaultCommand(hood.run(() ->
                        hood.setHoodAngle(Rotations.of(hood.getHoodPosition()))));
                }

                if (intake != null && indexer != null){
                        indexer.setDefaultCommand(new indexerDefault(indexer, () -> intake.isOut()));
                }

                configureDriveBindings();
                configureSharedBindings();
                if (turret!=null && vision!=null){
                        turret.setDefaultCommand(visionAutoAim);
                }


                drivetrain.configurePathPlanner();
                registerNamedCommands();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
                }

        

        //  DRIVE BINDINGS (always active)

        private void configureDriveBindings() {
                drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                drivetrain.registerTelemetry(logger::telemeterize);

        }

        

        //  OPERATOR BINDINGS}
        
        private void configureSharedBindings() {

                if (intake != null) {
                        operator.a().toggleOnTrue(new IntakeOutCommand(intake));
                        //operator.a().toggleOnTrue(new runRoller(intake));

                }

                if (indexer != null && hood != null && shooter != null && vision != null && turret != null) {

                operator.rightTrigger().whileTrue(Commands.defer(() -> {
                        
                        return new ShootCommand(indexer, hood, shooter,
                                visionAutoAim::getDistanceToHub,
                                visionAutoAim.getCalculator(),
                                visionAutoAim::isAimed,
                                visionAutoAim::currentlyPassing,
                                drivetrain, turret);
                }, Set.of(indexer, hood, shooter)));

                
        }

        if ( indexer != null) {

                operator.leftTrigger().whileTrue(
                        Commands.startEnd(
                                () -> indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL),
                                () -> indexer.stopIndexer(IndexerType.VERTICAL),
                                indexer));
        }
                if (turret != null) {

                operator.rightBumper().onTrue(Commands.runOnce(() -> {
                        turret.getTurretMotor().setPosition(0);
                        System.out.println("[TURRET] Zeroed at current position");
                }));
        }

                // CalibrationShootCommand — uncomment for interp table calibration sessions only
                // if (turret != null && hood != null && shooter != null && indexer != null && vision != null){
                // joystick.b().whileTrue(new CalibrationShootCommand(turret, hood, shooter, indexer, vision));
                // }
                
                // Turret setpoints (D-pad) — interrupts auto-aim default while held

                if (turret != null){
                        operator.povUp().whileTrue(turret.run(
                                () -> turret.moveTurret(Rotations.of(TurretConstants.TURRET_POS_FORWARD))));
                        operator.povDown().whileTrue(turret.run(
                                () -> turret.moveTurret(Rotations.of(TurretConstants.TURRET_POS_BACK))));
                        operator.povLeft().whileTrue(turret.run(
                                () -> turret.moveTurret(Rotations.of(TurretConstants.TURRET_POS_LEFT))));
                        operator.povRight().whileTrue(turret.run(
                                () -> turret.moveTurret(Rotations.of(TurretConstants.TURRET_POS_RIGHT))));
                }

                if (indexer != null && hood != null && shooter != null) {
                       operator.rightStick().whileTrue(new reverseIndexers(indexer , shooter));
                }    
                if (hood != null) 

                        operator.x().whileTrue(hood.run(
                                () -> hood.setHoodAngle(Rotations.of(TurretConstants.HOOD_POS_DOWN))));
                        operator.y().whileTrue(hood.run(
                                () -> hood.setHoodAngle(Rotations.of(TurretConstants.HOOD_POS_MID))));
                        operator.b().whileTrue(hood.run(
                                () -> hood.setHoodAngle(Rotations.of(TurretConstants.HOOD_POS_UP))));

                }
        

        //  NAMED COMMANDS (for PathPlanner event markers)
        private void registerNamedCommands() {
                if (intake != null) {
                        NamedCommands.registerCommand("intakeOut", new IntakeOutCommand(intake));
                        NamedCommands.registerCommand("intakeIn", new IntakeInCommand(intake));
                }

                if (indexer != null && hood != null && shooter != null && vision != null && turret != null){

                NamedCommands.registerCommand("shoot",
                        new ShootCommand(indexer, hood, shooter,
                                visionAutoAim::getDistanceToHub,
                                visionAutoAim.getCalculator(),
                                visionAutoAim::isAimed,
                                () -> false,
                                drivetrain, turret));
                }

                if (shooter!= null && indexer != null){

                NamedCommands.registerCommand("stopAll",
                        Commands.sequence(
                                Commands.runOnce(() -> shooter.stop(), shooter),
                                Commands.runOnce(() -> indexer.stop(), indexer)));

                }
                if (shooter != null && hood != null && vision != null && shooter != null && turret != null){

                NamedCommands.registerCommand("autoShoot",
                                new AutoShootCommand(shooter, hood,
                                        visionAutoAim::getDistanceToHub,
                                        visionAutoAim.getCalculator()));
                }

                if (indexer != null){

                NamedCommands.registerCommand("feedIndexers",
                        Commands.runEnd(
                                () -> {
                                        indexer.setIndexerSpeed(IndexerConstants.HORIZONTAL_INDEXER_SPEED, IndexerType.HORIZONTAL);
                                        indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL);
                                        indexer.setIndexerSpeed(IndexerConstants.UPWARD_INDEXER_SPEED, IndexerType.UPWARD);
                                },
                                () -> indexer.stop(),
                                indexer));
                NamedCommands.registerCommand("reverseIndexer", 
                        new reverseIndexers(indexer, shooter));
                        }

        }

        //  AUTONOMOUS
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
                // return null;
        }
}