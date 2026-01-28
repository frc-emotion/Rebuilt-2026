package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * AUTO-ALIGN command that centers the FRONT LEFT camera (aaranc) on an
 * AprilTag.
 * 
 * Logic:
 * - Auto-Align: Rotates robot to keep AprilTag centered in left camera view
 * - NOTE: User found that NEGATIVE rotation rate is required for their physical
 * setup.
 */
public class AimAtLeftCamera extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    // Field-centric drive request
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Simple proportional gain
    private static final double kP = 0.08;

    // Maximum rotation speed (radians per second)
    private static final double MAX_ROTATION_SPEED = 2.0;

    public AimAtLeftCamera(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("AimAtLeft/Active", true);
    }

    @Override
    public void execute() {
        double rotationRate = 0.0;

        // Get result from LEFT camera (aaranc)
        var result = vision.getLatestResultLeft();
        boolean hasTarget = (result != null && result.hasTargets());

        SmartDashboard.putBoolean("AimAtLeft/HasTarget", hasTarget);

        if (hasTarget) {
            double yawDegrees = result.getBestTarget().getYaw();
            int tagId = result.getBestTarget().getFiducialId();

            double rawRotation = kP * yawDegrees;
            rotationRate = MathUtil.clamp(rawRotation, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
            rotationRate = -rotationRate; // User's verified fix

            SmartDashboard.putNumber("AimAtLeft/TagID", tagId);
            SmartDashboard.putNumber("AimAtLeft/YawDeg", yawDegrees);
            SmartDashboard.putNumber("AimAtLeft/RotationRate", rotationRate);
            SmartDashboard.putBoolean("AimAtLeft/OnTarget", Math.abs(yawDegrees) < 2.0);
        } else {
            SmartDashboard.putNumber("AimAtLeft/TagID", -1);
            SmartDashboard.putNumber("AimAtLeft/YawDeg", 0);
            SmartDashboard.putNumber("AimAtLeft/RotationRate", 0);
            SmartDashboard.putBoolean("AimAtLeft/OnTarget", false);
        }

        drivetrain.setControl(
                driveRequest
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(rotationRate));
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("AimAtLeft/Active", false);
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
