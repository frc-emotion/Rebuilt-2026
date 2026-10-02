package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    private VisionConstants() {}

    public static final String shootCam = "Arducam_OV9281_USB_Camera";

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final Transform3d kRobotToCam = 
                new Transform3d(new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0, 0, Math.PI));

    public static Pose2d kBlueHubCenter = 
        new Pose2d(Units.inchesToMeters(182.97), Units.inchesToMeters(158.844) , Rotation2d.kZero);

    public static Pose2d kRedHubCenter = 
        new Pose2d(Units.inchesToMeters(469.115) , Units.inchesToMeters(158.844), Rotation2d.kZero);

    public static double turretDiffTolerance = 5/360;
}
