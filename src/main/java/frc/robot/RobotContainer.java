package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.CANID;
import frc.robot.commands.OdometryAutoAimCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurretAutoAimCommand;
import frc.robot.commands.indexer.runIndexer;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.turret.ManualShooterCommand;
import frc.robot.commands.turret.ManualTurretCommand;
import frc.robot.commands.turret.ManualHoodCommand;
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
        //  ROBOT MODE — change this ONE enum to switch operating modes
        // ================================================================
        public enum RobotMode {
                /** Joystick-only turret/hood, trigger = indexers + shooter. For bench testing. */
                MANUAL,
                /** Odometry-only auto-aim (no cameras). Turret tracks hub via atan2 on pose. */
                NO_VISION,
                /** Full vision + odometry Kalman-fused auto-aim. Requires cameras. */
                FULL_VISION
        }

        private static final RobotMode ACTIVE_MODE = RobotMode.FULL_VISION;

        // ================================================================
        //  DRIVE CONSTANTS
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

        @NotLogged
        public final Vision vision = (ACTIVE_MODE == RobotMode.FULL_VISION) ? new Vision() : null;

        public final Intake intake = new Intake(mechanismBus);
        public final Indexer indexer = new Indexer(mechanismBus);
        public final Turret turret = new Turret(mechanismBus);
        public final Hood hood = new Hood(mechanismBus);
        public final Shooter shooter = new Shooter(mechanismBus);

        // DISABLED hardware — kept as null so the references compile
        @NotLogged
        public final Climb climb = null;

        // ================================================================
        //  AUTO-AIM REFERENCES (null when mode doesn't use them)
        // ================================================================
        @NotLogged private TurretAutoAimCommand visionAutoAimCommand;
        @NotLogged private OdometryAutoAimCommand odomAutoAimCommand;
        @Logged private frc.robot.commands.TurretVisionTrackingTest visionTrackingTest;

        // ================================================================
        //  LOGGING & TUNING
        // ================================================================
        private final Telemetry logger = new Telemetry(MaxSpeed);
        @Logged public final FaultMonitor faultMonitor = new FaultMonitor();
        @Logged public final SuperstructureTuner tuner = new SuperstructureTuner();

        public RobotContainer() {
                edu.wpi.first.wpilibj.DriverStation.silenceJoystickConnectionWarning(true);
                System.out.println("[STARTUP] Active mode: " + ACTIVE_MODE);

                configureDriveBindings();

                switch (ACTIVE_MODE) {
                        case MANUAL     -> configureManualBindings();
                        case NO_VISION  -> configureNoVisionBindings();
                        case FULL_VISION -> configureFullVisionBindings();
                }

                configureIntakeBindings();
                registerMotorsForFaultMonitoring();
                tuner.setSubsystems(turret, hood, shooter, intake);

                if (vision != null && turret != null) {
                        vision.setTurretAngleSupplier(turret::getTurretPosition);
                }
        }

        /** @return true when FULL_VISION mode is active and vision should feed pose estimates. */
        public boolean isVisionPoseEstimationEnabled() {
                return ACTIVE_MODE == RobotMode.FULL_VISION && vision != null;
        }

        /**
         * Feeds vision pose estimates into the drivetrain's Kalman filter.
         * Called every cycle from Robot.robotPeriodic() when FULL_VISION is active.
         */
        public void updateVisionPoseEstimates() {
                if (vision == null) return;

                vision.getEstimatedPoseRight().ifPresent(estimate ->
                        drivetrain.addVisionMeasurement(
                                estimate.estimatedPose.toPose2d(),
                                estimate.timestampSeconds,
                                vision.getStdDevsRight()));

                vision.getEstimatedPoseLeft().ifPresent(estimate ->
                        drivetrain.addVisionMeasurement(
                                estimate.estimatedPose.toPose2d(),
                                estimate.timestampSeconds,
                                vision.getStdDevsLeft()));

                vision.getEstimatedPoseTurret().ifPresent(estimate ->
                        drivetrain.addVisionMeasurement(
                                estimate.estimatedPose.toPose2d(),
                                estimate.timestampSeconds,
                                vision.getStdDevsTurret()));
        }

        // ================================================================
        //  DRIVE (identical in all modes)
        // ================================================================
        private void configureDriveBindings() {
                drivetrain.setDefaultCommand(
                        drivetrain.applyRequest(() -> drive
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
        //  INTAKE (identical in all modes — operator A toggles in/out)
        // ================================================================
        private void configureIntakeBindings() {
                if (intake == null) return;
                operator.a().onTrue(Commands.defer(() -> {
                        if (intake.isOut()) {
                                return new IntakeInCommand(intake);
                        } else {
                                return new IntakeOutCommand(intake);
                        }
                }, Set.of(intake)));
        }

        // ================================================================
        //  MODE 1: MANUAL — pure joystick control
        //
        //  Operator right stick X → turret voltage
        //  Operator left stick Y  → hood voltage
        //  Operator right trigger → all 3 indexers + shooter (trigger-proportional)
        // ================================================================
        private void configureManualBindings() {
                turret.setDefaultCommand(new ManualTurretCommand(turret, () -> operator.getRightX()));
                hood.setDefaultCommand(new ManualHoodCommand(hood, () -> operator.getLeftY()));

                operator.rightTrigger().whileTrue(new ParallelCommandGroup(
                        new runIndexer(indexer),
                        new ManualShooterCommand(shooter, () -> operator.getRightTriggerAxis())));
        }

        // ================================================================
        //  MODE 2: NO_VISION — odometry-only auto-aim
        //
        //  Turret, hood, shooter all auto-set from odometry + interp tables.
        //  Operator right trigger → run all indexers (gated on aim).
        // ================================================================
        private void configureNoVisionBindings() {
                odomAutoAimCommand = new OdometryAutoAimCommand(drivetrain, turret, hood, shooter);
                turret.setDefaultCommand(odomAutoAimCommand);

                operator.rightTrigger().whileTrue(
                        new ShootCommand(indexer, odomAutoAimCommand));
        }

        // ================================================================
        //  MODE 3: FULL_VISION — odometry + vision Kalman-fused auto-aim
        //
        //  Vision feeds drivetrain pose estimator (Kalman filter in
        //  CommandSwerveDrivetrain). Turret camera additionally provides
        //  fine-grained tx correction on top of the odometry setpoint.
        //  Operator right trigger → run all indexers (gated on aim).
        // ================================================================
        private void configureFullVisionBindings() {
                visionAutoAimCommand = new TurretAutoAimCommand(drivetrain, vision, turret, hood, shooter);
                // Hold turret at current position by default (no auto-aim until odometry is calibrated).
                // Re-enable auto-aim as default when ready: turret.setDefaultCommand(visionAutoAimCommand);
                turret.setDefaultCommand(turret.run(() -> turret.moveTurret(
                        edu.wpi.first.units.Units.Rotations.of(turret.getTurretMotor().getPosition().getValueAsDouble()))));

                // Vision tracking test: turret tracks hub tag with velocity control.
                // Hold left bumper to test vision tracking independently.
                visionTrackingTest = new frc.robot.commands.TurretVisionTrackingTest(vision, turret, hood, shooter);
                operator.leftBumper().whileTrue(visionTrackingTest);

                // Turret position test setpoints (range: -0.58 to 0.20)
                operator.x().whileTrue(turret.run(() -> turret.moveTurret(
                        edu.wpi.first.units.Units.Rotations.of(-0.50))));  // near reverse limit
                operator.y().whileTrue(turret.run(() -> turret.moveTurret(
                        edu.wpi.first.units.Units.Rotations.of(-0.23))));    // center (forward)
                operator.b().whileTrue(turret.run(() -> turret.moveTurret(
                        edu.wpi.first.units.Units.Rotations.of(0.18))));   // near forward limit

                // Manual shoot test: right bumper revs shooter, right trigger feeds indexer
                operator.rightBumper().whileTrue(shooter.runEnd(
                        () -> shooter.setShooterSpeed(RotationsPerSecond.of(60)),
                        () -> shooter.stop()));
                operator.rightTrigger().whileTrue(new frc.robot.commands.indexer.runIndexer(indexer));
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
                                .withVelocityY(0)
                                .withRotationalRate(0))
                                .withTimeout(5.0),
                        drivetrain.applyRequest(() -> idle));
        }
}
