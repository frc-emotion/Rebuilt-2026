package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

@Logged
public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(RobotConstants.DRIVER_PORT);
    private final CommandXboxController operator = new CommandXboxController(RobotConstants.OPERATOR_PORT);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();
    public final Turret turret = new Turret();
    public final Hood hood = new Hood();
    public final Shooter shooter = new Shooter();

    public final StateMachine stateMachine = new StateMachine(
            drivetrain, intake, indexer, turret, hood, shooter,
            operator::getRightX,
            () -> -operator.getLeftY());

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(RobotConstants.DRIVE_MAX_SPEED_MPS * RobotConstants.DRIVE_TRANSLATION_DEADBAND_FRACTION)
            .withRotationalDeadband(RobotConstants.DRIVE_MAX_ANGULAR_RATE_RAD_PER_SEC * RobotConstants.DRIVE_ROTATION_DEADBAND_FRACTION)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    private final Telemetry telemetry = new Telemetry(RobotConstants.DRIVE_MAX_SPEED_MPS);

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive
                .withVelocityX(-driver.getLeftY() * RobotConstants.DRIVE_MAX_SPEED_MPS)
                .withVelocityY(-driver.getLeftX() * RobotConstants.DRIVE_MAX_SPEED_MPS)
                .withRotationalRate(-driver.getRightX() * RobotConstants.DRIVE_MAX_ANGULAR_RATE_RAD_PER_SEC)));

        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(telemetry::telemeterize);
    }

    private void configureOperatorBindings() {
        operator.a().onTrue(Commands.runOnce(stateMachine::toggleIntake));

        operator.rightTrigger().whileTrue(Commands.startEnd(
                () -> stateMachine.setShoot(true),
                () -> stateMachine.setShoot(false)));

        operator.rightStick().whileTrue(Commands.startEnd(
                () -> stateMachine.setClear(true),
                () -> stateMachine.setClear(false)));

        operator.povUp().onTrue(Commands.runOnce(() -> stateMachine.setHoodSetpoint(OperatorConstants.HOOD_PRESET_UP_ROT)));
        operator.povRight().onTrue(Commands.runOnce(() -> stateMachine.setHoodSetpoint(OperatorConstants.HOOD_PRESET_RIGHT_ROT)));
        operator.povLeft().onTrue(Commands.runOnce(() -> stateMachine.setHoodSetpoint(OperatorConstants.HOOD_PRESET_LEFT_ROT)));
        operator.povDown().onTrue(Commands.runOnce(() -> stateMachine.setHoodSetpoint(OperatorConstants.HOOD_PRESET_DOWN_ROT)));

        operator.back().onTrue(Commands.runOnce(() -> {
            turret.zero();
            stateMachine.setTurretSetpoint(0.0);
        }));
    }
}
