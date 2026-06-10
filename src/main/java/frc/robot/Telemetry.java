package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Swerve telemetry per the dashboard plan: pose + battery stay on NT (drive team needs the field
 * view to trust the new pose estimation); module-level detail goes ONLY to the hoot log via
 * SignalLogger. Replaces the stock CTRE Telemetry class's full NT firehose.
 */
public class Telemetry {
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private final StructPublisher<Pose2d> drivePose =
      inst.getTable("DriveState").getStructTopic("Pose", Pose2d.struct).publish();

  // Legacy Field2d emulation so Glass/Elastic render the robot on the field.
  private final NetworkTable poseTable = inst.getTable("Pose");
  private final DoubleArrayPublisher fieldPub = poseTable.getDoubleArrayTopic("robotPose").publish();
  private final StringPublisher fieldTypePub = poseTable.getStringTopic(".type").publish();

  private final DoublePublisher batteryVoltage =
      inst.getTable("MiscTelemetry").getDoubleTopic("BatteryVoltage").publish();

  /** Runs on the CTRE odometry thread — keep it allocation-light. */
  public void telemeterize(SwerveDriveState state) {
    drivePose.set(state.Pose);
    fieldTypePub.set("Field2d");
    fieldPub.set(new double[] {
      state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation().getDegrees()
    });
    batteryVoltage.set(RobotController.getBatteryVoltage());

    SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
    SignalLogger.writeStruct(
        "DriveState/Speeds", edu.wpi.first.math.kinematics.ChassisSpeeds.struct, state.Speeds);
    SignalLogger.writeStructArray(
        "DriveState/ModuleStates",
        edu.wpi.first.math.kinematics.SwerveModuleState.struct,
        state.ModuleStates);
    SignalLogger.writeStructArray(
        "DriveState/ModuleTargets",
        edu.wpi.first.math.kinematics.SwerveModuleState.struct,
        state.ModuleTargets);
    SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
    SignalLogger.writeDouble("MiscTelemetry/BatteryVoltage", RobotController.getBatteryVoltage());
  }
}
