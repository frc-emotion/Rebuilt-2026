package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * Thin adapter over the Phoenix Tuner X generated drivetrain (drivetrain policy: generated files
 * are frozen; everything the rest of the robot needs goes through here). The Superstructure and
 * bindings never touch CommandSwerveDrivetrain directly.
 */
public class Drive {
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final SwerveRequest.FieldCentric teleopRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(DriveConstants.kTranslationDeadband)
          .withRotationalDeadband(DriveConstants.kRotationDeadband)
          .withDriveRequestType(
              com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  /** Legacy default drive: all three axes negated, field-centric, open-loop voltage. */
  public Command teleopDriveCommand(
      DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
    return drivetrain.applyRequest(
        () ->
            teleopRequest
                .withVelocityX(-leftY.getAsDouble() * DriveConstants.kMaxSpeedMps)
                .withVelocityY(-leftX.getAsDouble() * DriveConstants.kMaxSpeedMps)
                .withRotationalRate(
                    -rightX.getAsDouble() * DriveConstants.kMaxAngularRateRadPerSec));
  }

  public Command brakeCommand() {
    return drivetrain.applyRequest(() -> brakeRequest);
  }

  public Command seedFieldCentricCommand() {
    return drivetrain.runOnce(drivetrain::seedFieldCentric);
  }

  /** CTRE-recommended Idle request while disabled (bind with ignoringDisable). */
  public Command idleWhileDisabledCommand() {
    return drivetrain.applyRequest(() -> idleRequest).ignoringDisable(true);
  }

  /**
   * Continuous Pigeon2 yaw in degrees — the turret gyro feedforward's signal. Same source and sign
   * as legacy (W3 is load-bearing); does NOT wrap at ±180 like pose heading does.
   */
  public double getContinuousYawDeg() {
    return drivetrain.getPigeon2().getYaw().getValueAsDouble();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return drivetrain.getState().Speeds;
  }

  public Pose2d getPose() {
    return drivetrain.getState().Pose;
  }

  /** Reset the pose estimate (used to seed a sim start pose; PathPlanner also resets here). */
  public void resetPose(Pose2d pose) {
    drivetrain.resetPose(pose);
  }

  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return drivetrain.samplePoseAt(timestampSeconds);
  }

  public void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    drivetrain.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
    drivetrain.registerTelemetry(telemetryFunction);
  }

  /** The subsystem handle for default-command installation and PathPlanner requirements. */
  public CommandSwerveDrivetrain subsystem() {
    return drivetrain;
  }

  /**
   * On-the-fly pathfinding to a field pose (the autonomy seam — see docs/live-steering.md). Returns
   * a PathPlanner command that computes a fresh navgrid-aware route from the current pose to {@code
   * target} and follows it; it requires the drivetrain, so scheduling it preempts the default
   * teleop command. The target is blue-origin; {@code pathfindToPoseFlipped} mirrors it for the red
   * alliance. Requires {@link #configureAutoBuilder()} to have run.
   */
  public Command driveToPose(Pose2d target, PathConstraints constraints) {
    return AutoBuilder.pathfindToPoseFlipped(target, constraints);
  }

  /**
   * PathPlanner AutoBuilder config, moved verbatim out of the generated class (drivetrain policy).
   * Call once after construction, before buildAutoChooser.
   */
  public void configureAutoBuilder() {
    SwerveRequest.ApplyRobotSpeeds pathRequest = new SwerveRequest.ApplyRobotSpeeds();
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> drivetrain.getState().Pose,
          drivetrain::resetPose,
          () -> drivetrain.getState().Speeds,
          (speeds, feedforwards) ->
              drivetrain.setControl(
                  pathRequest
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              new PIDConstants(DriveConstants.kPathTranslationP, 0.0, 0.0),
              new PIDConstants(DriveConstants.kPathRotationP, 0.0, 0.0)),
          config,
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          drivetrain);
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config — open PathPlanner GUI, go to Settings, "
              + "and configure your robot. Error: "
              + ex.getMessage(),
          ex.getStackTrace());
    }
  }
}
