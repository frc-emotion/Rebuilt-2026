package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Set;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
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
import frc.robot.commands.CalibrationShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurretAutoAimCommand;
import frc.robot.commands.climb.VoltageClimbCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.runRoller;
import frc.robot.generated.TunerConstants;
import frc.robot.logging.FaultMonitor;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

@Logged
public class RobotContainer {

        // ================================================================
        //  DRIVE
        // ================================================================
        private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        // ================================================================
        //  CONTROLLERS
        // ================================================================
        private final CommandXboxController joystick = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);
        private final CANBus mechanismBus = new CANBus("mechanisms");

        // ================================================================
        //  SUBSYSTEMS
        // ================================================================
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        // @NotLogged public final Vision vision = null;
        @Logged public final Vision vision = new Vision();
        // To re-enable intake: uncomment the line below and comment out the null line
        //public final Intake intake = null;
        public final Intake intake = new Intake(mechanismBus);
        public final Indexer indexer = new Indexer(mechanismBus);
        public final Turret turret = new Turret(mechanismBus);
        public final Hood hood = new Hood(mechanismBus);
        public final Shooter shooter = new Shooter(mechanismBus);
        public final Climb climb = new Climb(mechanismBus);

        @Logged private final TurretAutoAimCommand visionAutoAim;

        private final Telemetry logger = new Telemetry(MaxSpeed);
        @Logged public final FaultMonitor faultMonitor = new FaultMonitor();
        @NotLogged private final SendableChooser<Command> autoChooser;

        // Turret D-pad setpoints (intuitive relative to robot)
        // Up = forward (0°), Down = backward (-180°), Left = 90° left, Right = forward limit
        private static final double TURRET_POS_FORWARD  =  0.0;     //   0° (front of robot)
        private static final double TURRET_POS_RIGHT    =  0.25;    // +18° CW (forward limit)
        private static final double TURRET_POS_LEFT     = -0.250;   // -90° CCW (perpendicular left)
        private static final double TURRET_POS_BACK     = -0.500;   // -180° CCW (straight backward)

        // Hood setpoints (X/Y/B): range [0.0, 0.08]
        private static final double HOOD_POS_DOWN = 0.005;
        private static final double HOOD_POS_MID  = 0.040;
        private static final double HOOD_POS_UP   = 0.070;

        public RobotContainer() {
                edu.wpi.first.wpilibj.DriverStation.silenceJoystickConnectionWarning(true);

                visionAutoAim = new TurretAutoAimCommand(drivetrain, vision, turret,
                        () -> operator.getRightX(),
                        () -> operator.leftStick().getAsBoolean());

                hood.setDefaultCommand(hood.run(() ->
                        hood.setHoodAngle(Rotations.of(hood.getHoodPosition()))));

                if (intake != null) {
                        indexer.setDefaultCommand(indexer.run(() -> {
                                if (intake.isOut()) {
                                        indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED * 0.75, IndexerType.VERTICAL);
                                } else {
                                        indexer.stopIndexer(IndexerType.VERTICAL);
                                }
                        }));
                }

                configureDriveBindings();
                configureSharedBindings();
                configureClimbBindings();
                turret.setDefaultCommand(visionAutoAim);

                registerMotorsForFaultMonitoring();

                drivetrain.configurePathPlanner();
                registerNamedCommands();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        // ================================================================
        //  DRIVE BINDINGS (always active)
        // ================================================================
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

        // ================================================================
        //  CLIMB BINDINGS (driver controller)
        //
        //  Default command: VoltageClimbCommand reads driver triggers.
        //    Right trigger (held) = climb UP   (CCW, positive voltage)
        //    Left trigger  (held) = climb DOWN (CW, negative voltage)
        //    Both released         = gravity comp only (holds position via brake)
        // ================================================================
        private void configureClimbBindings() {
                if (climb != null) {
                        // Combined triggers: right = up (+), left = down (-)
                        climb.setDefaultCommand(new VoltageClimbCommand(climb,
                                () -> joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis()));
                }
                
        }

        // ================================================================
        //  OPERATOR BINDINGS}
        //
        //  A          = intake toggle
        //  Right trig = SHOOT (hood + shooter + indexers from interp table)
        //  Left trig  = vertical indexer only
        //  Right bump = zero turret encoder
        //  Right stick click = reverse indexers (unjam)
        //  D-pad      = turret setpoints
        //  X / Y / B  = hood setpoints
        // ================================================================
        private void configureSharedBindings() {
                if (intake != null) {
                        operator.a().toggleOnTrue(new IntakeOutCommand(intake));
                        //operator.a().toggleOnTrue(new runRoller(intake));

                }

                operator.rightTrigger().whileTrue(Commands.defer(() -> {
                        return new ShootCommand(indexer, hood, shooter,
                                visionAutoAim::getDistanceToHub,
                                visionAutoAim.getCalculator(),
                                visionAutoAim::isAimed);
                }, Set.of(indexer, hood, shooter)));

                operator.leftTrigger().whileTrue(
                        Commands.startEnd(
                                () -> indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL),
                                () -> indexer.stopIndexer(IndexerType.VERTICAL),
                                indexer));

                operator.rightBumper().onTrue(Commands.runOnce(() -> {
                        turret.getTurretMotor().setPosition(0);
                        System.out.println("[TURRET] Zeroed at current position");
                }));

                // CalibrationShootCommand — uncomment for interp table calibration sessions only
                joystick.b().whileTrue(new CalibrationShootCommand(turret, hood, shooter, indexer, vision));

                // Turret setpoints (D-pad) — interrupts auto-aim default while held
                operator.povUp().whileTrue(turret.run(
                        () -> turret.moveTurret(Rotations.of(TURRET_POS_FORWARD))));
                operator.povDown().whileTrue(turret.run(
                        () -> turret.moveTurret(Rotations.of(TURRET_POS_BACK))));
                operator.povLeft().whileTrue(turret.run(
                        () -> turret.moveTurret(Rotations.of(TURRET_POS_LEFT))));
                operator.povRight().whileTrue(turret.run(
                        () -> turret.moveTurret(Rotations.of(TURRET_POS_RIGHT))));

                operator.rightStick().whileTrue(
                        Commands.startEnd(
                                () -> {
                                        indexer.setIndexerSpeed(-IndexerConstants.HORIZONTAL_INDEXER_SPEED * 0.5, IndexerType.HORIZONTAL);
                                        indexer.setIndexerSpeed(-IndexerConstants.VERTICAL_INDEXER_SPEED * 0.5, IndexerType.VERTICAL);
                                        indexer.setIndexerSpeed(-IndexerConstants.UPWARD_INDEXER_SPEED * 0.5, IndexerType.UPWARD);
                                        shooter.setShooterSpeed(RotationsPerSecond.of(-TurretConstants.MAX_SHOOTER_RPS));
                                },
                                () -> indexer.stop(),
                                indexer));

                operator.x().whileTrue(hood.run(
                        () -> hood.setHoodAngle(Rotations.of(HOOD_POS_DOWN))));
                operator.y().whileTrue(hood.run(
                        () -> hood.setHoodAngle(Rotations.of(HOOD_POS_MID))));
                operator.b().whileTrue(hood.run(
                        () -> hood.setHoodAngle(Rotations.of(HOOD_POS_UP))));
        }

        // ================================================================
        //  FAULT MONITORING
        // ================================================================
        private void registerMotorsForFaultMonitoring() {
                for (int i = 0; i < 4; i++) {
                        var module = drivetrain.getModule(i);
                        faultMonitor.register(CANID.SWERVE_IDS[i][0], module.getDriveMotor());
                        faultMonitor.register(CANID.SWERVE_IDS[i][1], module.getSteerMotor());
                }
                if (indexer != null) {
                        faultMonitor.register(CANID.HORIZONTAL_INDEXER, indexer.getHorizontalMotor());
                        faultMonitor.register(CANID.VERTICAL_INDEXER, indexer.getVerticalMotor());
                        faultMonitor.register(CANID.UPWARD_INDEXER, indexer.getUpwardMotor());
                }
                if (intake != null) {
                        faultMonitor.register(CANID.INTAKE_MOTOR, intake.getIntakeMotor());
                        faultMonitor.register(CANID.ROLLER_MOTOR, intake.getRollerMotor());
                }
                if (turret != null) faultMonitor.register(CANID.TURRET_ROTATION, turret.getTurretMotor());
                if (hood != null)   faultMonitor.register(CANID.TURRET_ANGLE, hood.getHoodMotor());
                if (shooter != null) faultMonitor.register(CANID.SHOOTER_WHEEL, shooter.getShooterMotor());
                if (climb != null) {
                    faultMonitor.register(CANID.CLIMB_MOTOR, climb.getClimbMotor());
                }
        }

        // ================================================================
        //  NAMED COMMANDS (for PathPlanner event markers)
        // ================================================================
        private void registerNamedCommands() {
                // if (intake != null) {
                //         NamedCommands.registerCommand("intakeOut", new IntakeOutCommand(intake));
                //         NamedCommands.registerCommand("intakeIn", new IntakeInCommand(intake));
                // }

                // NamedCommands.registerCommand("shoot",
                //         new ShootCommand(indexer, hood, shooter,
                //                 visionAutoAim::getDistanceToHub,
                //                 visionAutoAim.getCalculator(),
                //                 visionAutoAim::isAimed));

                // NamedCommands.registerCommand("stopAll",
                //         Commands.parallel(
                //                 Commands.runOnce(() -> shooter.stop(), shooter),
                //                 Commands.runOnce(() -> indexer.stop(), indexer)));
        }

        // ================================================================
        //  AUTONOMOUS
        // ================================================================
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
