package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.*;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.CANID;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;
import frc.robot.commands.CalibrationShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurretAutoAimCommand;
import frc.robot.commands.climb.VoltageClimbCommand;
import frc.robot.commands.intake.IntakeInCommand;
import frc.robot.commands.intake.IntakeOutCommand;
// import frc.robot.commands.turret.ManualTurretCommand; // replaced by unified TurretAutoAimCommand
import frc.robot.generated.TunerConstants;
import frc.robot.logging.FaultMonitor;
// import frc.robot.util.SuperstructureTuner; // REMOVED — tuner defaults conflicted with TurretConstants
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
        //  ROBOT MODE — unified: turret auto-switches between MANUAL and TRACKING
        //  based on whether the turret camera sees a hub tag.
        // ================================================================

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
        @NotLogged public final Vision vision = new Vision();
        public final Intake intake = new Intake(mechanismBus);
        public final Indexer indexer = new Indexer(mechanismBus);
        public final Turret turret = new Turret(mechanismBus);
        public final Hood hood = new Hood(mechanismBus);
        public final Shooter shooter = new Shooter(mechanismBus);
        public final Climb climb = new Climb(mechanismBus);

        // ================================================================
        //  AUTO-AIM COMMANDS (created once, swapped as default on mode change)
        // ================================================================
        @Logged private final TurretAutoAimCommand visionAutoAim;

        // ================================================================
        //  LOGGING & TUNING
        // ================================================================
        private final Telemetry logger = new Telemetry(MaxSpeed);
        @Logged public final FaultMonitor faultMonitor = new FaultMonitor();
        // @Logged public final SuperstructureTuner tuner = new SuperstructureTuner(); // REMOVED — use TurretConstants directly
        @NotLogged private final SendableChooser<Command> autoChooser;

        // Turret setpoints (D-pad): clean degree values for testing
        // Soft limits: -0.74 (reverse) to +0.05 (forward) — 270° range, mostly leftward
        // Up=0° (forward), Right=+18° CW (near fwd limit), Left=-180° (backward), Down=-90° (perpendicular left)
        private static final double TURRET_POS_CENTER   =  0.0;     //   0° (forward)
        private static final double TURRET_POS_RIGHT    =  0.05;    // +18° CW  (forward limit)
        private static final double TURRET_POS_LEFT     = -0.500;   // -180° CCW (straight backward)
        private static final double TURRET_POS_FAR_LEFT = -0.250;   // -90° CCW (perpendicular left)

        // Hood setpoints (X/Y/B): range [0.0, 0.08]
        private static final double HOOD_POS_DOWN = 0.005;
        private static final double HOOD_POS_MID  = 0.040;
        private static final double HOOD_POS_UP   = 0.070;

        public RobotContainer() {
                edu.wpi.first.wpilibj.DriverStation.silenceJoystickConnectionWarning(true);

                visionAutoAim = new TurretAutoAimCommand(drivetrain, vision, turret, () -> operator.getRightX());

                // Hood default: always hold last commanded position via MotionMagic
                hood.setDefaultCommand(hood.run(() ->
                        hood.setHoodAngle(Rotations.of(hood.getHoodPosition()))));

                indexer.setDefaultCommand(indexer.run(() -> {
                        if (intake.isOut()) {
                                indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED * 0.75, IndexerType.VERTICAL);
                        } else {
                                indexer.stopIndexer(IndexerType.VERTICAL);
                        }
                }));

                configureDriveBindings();
                configureSharedBindings();
                configureClimbBindings();
                turret.setDefaultCommand(visionAutoAim);

                registerMotorsForFaultMonitoring();
                // tuner.setSubsystems(turret, hood, shooter, intake); // REMOVED — SuperstructureTuner disabled
                vision.setTurretAngleSupplier(turret::getTurretPosition);

                // PathPlanner: configure path-following, register named commands, build auto chooser
                drivetrain.configurePathPlanner();
                registerNamedCommands();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

                System.out.println("[STARTUP] Turret unified command active (MANUAL/TRACKING)");
        }

        // ================================================================
        //  VISION POSE ESTIMATION — disabled, only turret cam active for tracking
        // ================================================================
        public boolean isVisionPoseEstimationEnabled() {
                return false;
        }

        public void updateVisionPoseEstimates() {
                // All drivebase cameras disabled. Turret camera is used for
                // turret tracking + distance only, not pose estimation.
                // Re-enable here when drivebase cameras come back online.
        }

        // ================================================================
        //  MODE SWITCHING — removed. Turret auto-switches MANUAL ↔ TRACKING
        //  based on turret camera seeing a hub tag.
        // ================================================================

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

                // -- Calibration shoot REMOVED from driver B — accidental press takes over all mechanisms --
                // joystick.b().whileTrue(new CalibrationShootCommand(turret, hood, shooter, indexer));

                // -- Seed pose facing hub (driver Y): sets odometry so hub is directly ahead --
                // Places robot ~4m in front of red hub center, facing +X (toward hub).
                // SEEKING mode then aims turret to 0° (forward), aligning camera with the tag.
                joystick.y().onTrue(Commands.runOnce(() -> {
                        var hubCenter = frc.robot.Constants.VisionConstants.RED_HUB_CENTER;
                        drivetrain.resetPose(new edu.wpi.first.math.geometry.Pose2d(
                                hubCenter.getX() - 4.0, hubCenter.getY(),
                                new Rotation2d(0)));
                        System.out.println("[POSE] Seeded facing red hub at ~4m distance");
                }));

                drivetrain.registerTelemetry(logger::telemeterize);

                // -- SysId characterization REMOVED — was on driver D-pad, dangerous during matches --
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
                        operator.a().toggleOnTrue(new IntakeOutCommand(intake));
                }

                // -- SHOOT (right trigger): ShootCommand handles hood + shooter + indexers --
                // Always use interp-table ShootCommand so hood/shooter adjust with
                // distance even if turret transitions between MANUAL and TRACKING
                // mid-shot. getEffectiveDistance() persists the last seen camera
                // distance, so hood still gets a valid setpoint after brief tag loss.
                // Falls back to minimum-range values if no tag has ever been seen (dist ≈ 0).
                operator.rightTrigger().whileTrue(Commands.defer(() -> {
                        return new ShootCommand(indexer, hood, shooter,
                                visionAutoAim::getEffectiveDistance,
                                visionAutoAim.getCalculator(),
                                visionAutoAim::isAimed,
                                visionAutoAim::getShooterRPSOffset);
                }, Set.of(indexer, hood, shooter)));

                // -- Vertical indexer only (left trigger) --
                operator.leftTrigger().whileTrue(
                        Commands.startEnd(
                                () -> indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL),
                                () -> indexer.stopIndexer(IndexerType.VERTICAL),
                                indexer));

                // -- Intake irrigate (left bumper): oscillate to dislodge stuck balls --
                // if (intake != null) {
                //         operator.leftBumper().whileTrue(new IntakeIrrigateCommand(intake));
                // }

                // -- Zero turret (right bumper) --
                operator.rightBumper().onTrue(Commands.runOnce(() -> {
                        turret.getTurretMotor().setPosition(0);
                        System.out.println("[TURRET] Zeroed at current position");
                }));

                // -- Calibration shoot (driver B): hold to shoot with Elastic-tunable values --
                joystick.b().whileTrue(new CalibrationShootCommand(turret, hood, shooter, indexer));

                // -- Turret setpoints (D-pad) — interrupt default while held --
                operator.povUp().whileTrue(turret.run(
                        () -> turret.moveTurret(Rotations.of(TURRET_POS_CENTER))));
                operator.povRight().whileTrue(turret.run(
                        () -> turret.moveTurret(Rotations.of(TURRET_POS_RIGHT))));
                operator.povLeft().whileTrue(turret.run(
                        () -> turret.moveTurret(Rotations.of(TURRET_POS_LEFT))));
                operator.povDown().whileTrue(turret.run(
                        () -> turret.moveTurret(Rotations.of(TURRET_POS_FAR_LEFT))));

                // -- Reverse indexers (right stick click): clear jams while held --
                operator.rightStick().whileTrue(
                        Commands.startEnd(
                                () -> {
                                        indexer.setIndexerSpeed(-IndexerConstants.HORIZONTAL_INDEXER_SPEED * 0.5, IndexerType.HORIZONTAL);
                                        indexer.setIndexerSpeed(-IndexerConstants.VERTICAL_INDEXER_SPEED * 0.5, IndexerType.VERTICAL);
                                        indexer.setIndexerSpeed(-IndexerConstants.UPWARD_INDEXER_SPEED * 0.5, IndexerType.UPWARD);
                                },
                                () -> indexer.stop(),
                                indexer));

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
                if (intake != null) {
                        faultMonitor.register(CANID.INTAKE_MOTOR, intake.getIntakeMotor());
                        faultMonitor.register(CANID.ROLLER_MOTOR, intake.getRollerMotor());
                }
                if (turret != null) faultMonitor.register(CANID.TURRET_ROTATION, turret.getTurretMotor());
                if (hood != null)   faultMonitor.register(CANID.TURRET_ANGLE, hood.getHoodMotor());
                if (shooter != null) faultMonitor.register(CANID.SHOOTER_WHEEL, shooter.getShooterMotor());
                if (climb != null) {
                    faultMonitor.register(CANID.CLIMB_FOLLOWER, climb.getClimbMotor());
                }
        }

        // ================================================================
        //  NAMED COMMANDS (for PathPlanner event markers)
        // ================================================================
        private void registerNamedCommands() {
                // -- Intake --
                NamedCommands.registerCommand("intakeOut",
                        new IntakeOutCommand(intake));
                NamedCommands.registerCommand("intakeIn",
                        new IntakeInCommand(intake));

                // -- Shoot (vision): hood + shooter from interp tables via turret auto-aim,
                //    all 3 indexers fire only when aimed AND shooter at setpoint --
                NamedCommands.registerCommand("shoot",
                        new ShootCommand(indexer, hood, shooter,
                                visionAutoAim::getEffectiveDistance,
                                visionAutoAim.getCalculator(),
                                visionAutoAim::isAimed,
                                visionAutoAim::getShooterRPSOffset));

                // -- Safety: stop all mechanisms --
                NamedCommands.registerCommand("stopAll",
                        Commands.parallel(
                                Commands.runOnce(() -> shooter.stop(), shooter),
                                Commands.runOnce(() -> indexer.stop(), indexer),
                                new IntakeInCommand(intake)));
        }

        // ================================================================
        //  AUTONOMOUS
        // ================================================================
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
