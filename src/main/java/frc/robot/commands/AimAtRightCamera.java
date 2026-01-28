package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * AUTO-ALIGN command that centers the FRONT RIGHT camera (mugilanr) on an
 * AprilTag
 * while allowing the driver to strafe and drive forward/backward.
 * 
 * Logic:
 * - Auto-Align: Rotates robot to keep AprilTag centered in right camera view
 * - Driver Control: Joystick X/Y inputs translate the robot (Robot-Centric)
 * - Deadband: Applied to driver inputs to prevent drift
 * 
 * Documentation Fact-Check (PhotonVision + WPILib):
 * - PhotonVision getYaw(): Positive = Target is LEFT of center
 * - WPILib Rotation: Positive = Counter-Clockwise (Left Turn)
 * - Alignment Logic: If target is Left (+Yaw), we need to turn Left (+Rotation)
 * to bring the target to center.
 * - NOTE: User found that NEGATIVE rotation rate is required for their physical
 * setup.
 * We respect this verified reality: rotationRate = - (kP * yaw)
 */
public class AimAtRightCamera extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    // Field-centric drive request (Easier for driver)
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Simple proportional gain
    private static final double kP = 0.08;

    // Maximum rotation speed (radians per second)
    private static final double MAX_ROTATION_SPEED = 2.0;

    /**
     * @param drivetrain      Swerve drivetrain
     * @param vision          Vision subsystem
     * @param forwardSupplier Joystick input for forward/back (Robot-Centric X)
     */
    public AimAtRightCamera(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("AimAtRight/Active", true);
    }

    @Override
    public void execute() {
        double rotationRate = 0.0;

        // 2. Calculate Auto-Align Rotation
        var result = vision.getLatestResultRight();
        boolean hasTarget = (result != null && result.hasTargets());

        SmartDashboard.putBoolean("AimAtRight/HasTarget", hasTarget);

        if (hasTarget) {
            // Get yaw (Positive = Left)
            double yawDegrees = result.getBestTarget().getYaw();
            int tagId = result.getBestTarget().getFiducialId();

            // Calculate rotation rate
            double rawRotation = kP * yawDegrees;

            // Clamp speed
            rotationRate = MathUtil.clamp(rawRotation, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

            // Apply User's fix (Negative sign)
            // Even though docs say +Yaw needs +Rotation to face it,
            // User confirmed -Rotation works for theirs bot.
            rotationRate = -rotationRate;

            // Telemetry
            SmartDashboard.putNumber("AimAtRight/TagID", tagId);
            SmartDashboard.putNumber("AimAtRight/YawDeg", yawDegrees);
            SmartDashboard.putNumber("AimAtRight/RotationRate", rotationRate);
            SmartDashboard.putBoolean("AimAtRight/OnTarget", Math.abs(yawDegrees) < 2.0);
        } else {
            // No target: No rotation
            SmartDashboard.putNumber("AimAtRight/TagID", -1);
            SmartDashboard.putNumber("AimAtRight/YawDeg", 0);
            SmartDashboard.putNumber("AimAtRight/RotationRate", 0);
            SmartDashboard.putBoolean("AimAtRight/OnTarget", false);
        }

        // 3. Apply Control (Translation + Rotation)
        drivetrain.setControl(
                driveRequest
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(rotationRate));

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("AimAtRight/Active", false);

        // Stop
        drivetrain.setControl(
                driveRequest
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
