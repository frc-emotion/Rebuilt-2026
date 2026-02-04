// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.CANID;
import frc.robot.commands.AimAtLeftCamera;
import frc.robot.commands.AimAtRightCamera;
import frc.robot.generated.TunerConstants;
import frc.robot.logging.FaultMonitor;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.commands.climb.ClimbSequenceCommand;

/**
 * Container for robot subsystems and commands.
 * 
 * <p>
 * All subsystems are instantiated here and automatically logged via Epilogue.
 */
@Logged
public class RobotContainer {
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final CommandXboxController joystick = new CommandXboxController(0);

        // ===== SUBSYSTEMS (all automatically logged via Epilogue) =====
        // Set to null to disable subsystems that don't have hardware connected
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Vision vision = new Vision();
        public final ClimbSubsystem climb = new ClimbSubsystem();
        // public final Intake intake = new Intake();
        // public final Indexer indexer = new Indexer();
        // public final Turret turret = new Turret();



        public final Intake intake = null; // Disabled: no hardware connected
        public final Indexer indexer = null; // Disabled: no hardware connected
        public final Turret turret = null; // Disabled: no hardware connected

        // ===== LOGGING & MONITORING =====
        private final Telemetry logger = new Telemetry(MaxSpeed);
        @Logged
        public final FaultMonitor faultMonitor = new FaultMonitor();

        public RobotContainer() {
                configureBindings();
                registerMotorsForFaultMonitoring();
        }

        /**
         * Registers all motors with the FaultMonitor for fault detection.
         * Null-safe - skips subsystems that are disabled.
         */
        private void registerMotorsForFaultMonitoring() {
                // Swerve drive motors (always enabled)
                for (int i = 0; i < 4; i++) {
                        var module = drivetrain.getModule(i);
                        faultMonitor.register(CANID.SWERVE_IDS[i][0], module.getDriveMotor());
                        faultMonitor.register(CANID.SWERVE_IDS[i][1], module.getSteerMotor());
                }

                // Intake motors (if enabled)
                if (intake != null) {
                        faultMonitor.register(CANID.INTAKE_MOTOR, intake.getIntakeMotor());
                        faultMonitor.register(CANID.ROLLER_MOTOR, intake.getRollerMotor());
                }

                // Indexer motors (if enabled)
                if (indexer != null) {
                        faultMonitor.register(CANID.HORIZONTAL_INDEXER, indexer.getHorizontalMotor());
                        faultMonitor.register(CANID.VERTICAL_INDEXER, indexer.getVerticalMotor());
                        faultMonitor.register(CANID.UPWARD_INDEXER, indexer.getUpwardMotor());
                }

                // Turret motors (if enabled)
                if (turret != null) {
                        faultMonitor.register(CANID.SHOOTER_WHEEL, turret.getShooterMotor());
                        faultMonitor.register(CANID.TURRET_ROTATION, turret.getTurretMotor());
                        faultMonitor.register(CANID.TURRET_ANGLE, turret.getHoodMotor());
                }
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                                   // forward
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // Y
                                                                                                                   // (forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                                                // negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // RIGHT TRIGGER: Aim at AprilTag using FRONT RIGHT camera (mugilanr)
                joystick.rightTrigger().whileTrue(
                                new AimAtRightCamera(drivetrain, vision));

                // LEFT TRIGGER: Aim at AprilTag using FRONT LEFT camera (aaranc)
                joystick.leftTrigger().whileTrue(
                                new AimAtLeftCamera(drivetrain, vision));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
                joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        /**
         * Updates vision pose estimates and feeds them to the drivetrain.
         * Call this from Robot.periodic().
         */
        public void updateVisionPoseEstimates() {
                // Update drivetrain with vision measurements from both cameras
                vision.getEstimatedPoseRight().ifPresent(estimate -> {
                        drivetrain.addVisionMeasurement(
                                        estimate.estimatedPose.toPose2d(),
                                        estimate.timestampSeconds,
                                        vision.getStdDevsRight());
                });

                vision.getEstimatedPoseLeft().ifPresent(estimate -> {
                        drivetrain.addVisionMeasurement(
                                        estimate.estimatedPose.toPose2d(),
                                        estimate.timestampSeconds,
                                        vision.getStdDevsLeft());
                });
        }

        public Command getAutonomousCommand() {
                // Simple drive forward auton
                final var idle = new SwerveRequest.Idle();
                return Commands.sequence(
                                // Reset our field centric heading to match the robot
                                // facing away from our alliance station wall (0 deg).
                                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                                // Then slowly drive forward (away from us) for 5 seconds.
                                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                                                .withVelocityY(0)
                                                .withRotationalRate(0))
                                                .withTimeout(5.0),
                                // Finally idle for the rest of auton
                                drivetrain.applyRequest(() -> idle));
        }
}
