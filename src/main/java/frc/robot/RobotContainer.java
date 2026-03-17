package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.*;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.CANID;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurretAutoAimCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeIrrigateCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.turret.ManualTurretCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.logging.FaultMonitor;
import frc.robot.util.SuperstructureTuner;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

@Logged
public class RobotContainer {

        // ================================================================
        //  ROBOT MODE — toggleable at runtime via operator left stick click
        // ================================================================
        public enum RobotMode { MANUAL, FULL_VISION }

        @Logged(importance = Logged.Importance.CRITICAL)
        private RobotMode activeMode = RobotMode.MANUAL;

        // ================================================================
        //  DRIVE
        // ================================================================
        private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
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
        @NotLogged public final Vision vision = new Vision();
        public final Intake intake = new Intake(mechanismBus);
        public final Indexer indexer = new Indexer(mechanismBus);
        public final Turret turret = new Turret(mechanismBus);
        public final Hood hood = new Hood(mechanismBus);
        public final Shooter shooter = new Shooter(mechanismBus);
        @NotLogged public final Climb climb = null;

        // ================================================================
        //  AUTO-AIM COMMANDS (created once, swapped as default on mode change)
        // ================================================================
        @NotLogged private final TurretAutoAimCommand visionAutoAim;
        @NotLogged private final ManualTurretCommand manualTurret;

        // ================================================================
        //  LOGGING & TUNING
        // ================================================================
        private final Telemetry logger = new Telemetry(MaxSpeed);
        @Logged public final FaultMonitor faultMonitor = new FaultMonitor();
        @Logged public final SuperstructureTuner tuner = new SuperstructureTuner();

        // ================================================================
        //  MANUAL MODE CONSTANTS
        // ================================================================
        private static final double MANUAL_SHOOTER_RPS = 40.0;

        // Turret setpoints (D-pad): 0=forward, spread across range [-0.58, 0.20]
        private static final double TURRET_POS_CENTER = 0.0;
        private static final double TURRET_POS_RIGHT  = 0.15;
        private static final double TURRET_POS_LEFT   = -0.20;
        private static final double TURRET_POS_FAR_LEFT = -0.45;

        // Hood setpoints (X/Y/B): range [0.0, 0.08]
        private static final double HOOD_POS_DOWN = 0.005;
        private static final double HOOD_POS_MID  = 0.040;
        private static final double HOOD_POS_UP   = 0.070;

        public RobotContainer() {
                edu.wpi.first.wpilibj.DriverStation.silenceJoystickConnectionWarning(true);

                visionAutoAim = new TurretAutoAimCommand(drivetrain, vision, turret);
                manualTurret = new ManualTurretCommand(turret, () -> operator.getRightX());

                // Hood default: always hold last commanded position via MotionMagic
                hood.setDefaultCommand(hood.run(() ->
                        hood.setHoodAngle(Rotations.of(hood.getHoodMotor().getPosition().getValueAsDouble()))));

                configureDriveBindings();
                configureSharedBindings();
                applyMode(activeMode);

                registerMotorsForFaultMonitoring();
                tuner.setSubsystems(turret, hood, shooter, intake);
                vision.setTurretAngleSupplier(turret::getTurretPosition);

                System.out.println("[STARTUP] Mode: " + activeMode);
        }

        // ================================================================
        //  VISION POSE ESTIMATION (called from Robot.robotPeriodic)
        // ================================================================
        public boolean isVisionPoseEstimationEnabled() {
                return activeMode == RobotMode.FULL_VISION;
        }

        public void updateVisionPoseEstimates() {
                vision.getEstimatedPoseRight().ifPresent(est ->
                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(),
                                est.timestampSeconds, vision.getStdDevsRight()));
                vision.getEstimatedPoseLeft().ifPresent(est ->
                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(),
                                est.timestampSeconds, vision.getStdDevsLeft()));
                vision.getEstimatedPoseTurret().ifPresent(est ->
                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(),
                                est.timestampSeconds, vision.getStdDevsTurret()));
        }

        // ================================================================
        //  MODE SWITCHING — operator left stick click toggles modes
        // ================================================================
        private void toggleMode() {
                applyMode(activeMode == RobotMode.MANUAL ? RobotMode.FULL_VISION : RobotMode.MANUAL);
        }

        private void applyMode(RobotMode mode) {
                activeMode = mode;
                Command currentDefault = turret.getDefaultCommand();
                if (currentDefault != null) currentDefault.cancel();

                switch (mode) {
                        case MANUAL      -> turret.setDefaultCommand(manualTurret);
                        case FULL_VISION -> turret.setDefaultCommand(visionAutoAim);
                }
                System.out.println("[MODE] Switched to " + mode);
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
        //  SHARED BINDINGS (active in ALL modes)
        //
        //  A          = intake toggle
        //  Right trig = SHOOT (indexers + shooter + hood from interp table)
        //  Left trig  = vertical indexer only
        //  Left bump  = intake irrigate (oscillate to unstick balls)
        //  Right bump = zero turret
        //  Left stick  = toggle mode (MANUAL ↔ FULL_VISION)
        //  D-pad      = turret setpoints (MANUAL: override joystick,
        //               FULL_VISION: temporarily override auto-aim while held)
        //  X / Y / B  = hood setpoints
        // ================================================================
        private void configureSharedBindings() {
                // -- Intake toggle (A) --
                if (intake != null) {
                        operator.a().onTrue(Commands.defer(() -> {
                                if (intake.isOut()) return new IntakeInCommand(intake);
                                else return new IntakeOutCommand(intake);
                        }, Set.of(intake)));
                }

                // -- SHOOT (right trigger): ShootCommand handles hood + shooter + indexers --
                operator.rightTrigger().whileTrue(Commands.defer(() -> {
                        if (activeMode == RobotMode.MANUAL) {
                                return new ShootCommand(indexer, hood, shooter, MANUAL_SHOOTER_RPS);
                        } else {
                                return new ShootCommand(indexer, hood, shooter,
                                        visionAutoAim::getDistanceToHub,
                                        visionAutoAim.getCalculator(),
                                        visionAutoAim::isAimed);
                        }
                }, Set.of(indexer, hood, shooter)));

                // -- Vertical indexer only (left trigger) --
                operator.leftTrigger().whileTrue(
                        Commands.startEnd(
                                () -> indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL),
                                () -> indexer.setIndexerSpeed(0, IndexerType.VERTICAL),
                                indexer));

                // -- Intake irrigate (left bumper): oscillate to dislodge stuck balls --
                if (intake != null) {
                        operator.leftBumper().whileTrue(new IntakeIrrigateCommand(intake));
                }

                // -- Zero turret (right bumper) --
                operator.rightBumper().onTrue(Commands.runOnce(() -> {
                        turret.getTurretMotor().setPosition(0);
                        System.out.println("[TURRET] Zeroed at current position");
                }));

                // -- Toggle mode (left stick click) --
                operator.leftStick().onTrue(Commands.runOnce(this::toggleMode));

                // -- Turret setpoints (D-pad) — interrupt default while held --
                operator.povUp().whileTrue(turret.run(
                        () -> turret.moveTurret(Rotations.of(TURRET_POS_CENTER))));
                operator.povRight().whileTrue(turret.run(
                        () -> turret.moveTurret(Rotations.of(TURRET_POS_RIGHT))));
                operator.povLeft().whileTrue(turret.run(
                        () -> turret.moveTurret(Rotations.of(TURRET_POS_LEFT))));
                operator.povDown().whileTrue(turret.run(
                        () -> turret.moveTurret(Rotations.of(TURRET_POS_FAR_LEFT))));

                // -- Hood setpoints (X / Y / B) --
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
                if (turret != null) faultMonitor.register(CANID.TURRET_ROTATION, turret.getTurretMotor());
                if (hood != null)   faultMonitor.register(CANID.TURRET_ANGLE, hood.getHoodMotor());
                if (shooter != null) faultMonitor.register(CANID.SHOOTER_WHEEL, shooter.getShooterMotor());
                // if (climb != null) {
                //     faultMonitor.register(CANID.CLIMB_LEADER, climb.getLeaderMotor());
                //     faultMonitor.register(CANID.CLIMB_FOLLOWER, climb.getFollowerMotor());
                // }
        }

        // ================================================================
        //  AUTONOMOUS
        // ================================================================
        public Command getAutonomousCommand() {
                final var idle = new SwerveRequest.Idle();
                return Commands.sequence(
                        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                        drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                                .withVelocityY(0).withRotationalRate(0))
                                .withTimeout(5.0),
                        drivetrain.applyRequest(() -> idle));
        }
}
