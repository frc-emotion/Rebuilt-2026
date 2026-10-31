package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    private VisionConstants() {}

    public static final String shootCam = "Arducam_OV9281_USB_Camera";

    public static final String leftCam = "Arducam_OV9281_USB_Camera (1)";

    public static final double maxAmbiguity = 0.2;

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final Transform3d kRobotToCam = 
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(-13.625),//-0.5
                        Units.inchesToMeters(-7.875),//-0.5, //0.0, //before postiive see if fixes and allat
                        Units.inchesToMeters(14.52)),//0.5
                    new Rotation3d(0, 0, Math.PI));

    public static final Transform3d kRobotToLeftCam =
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(-3.197),
                        Units.inchesToMeters(13.625),
                        Units.inchesToMeters(10)),
                    new Rotation3d(0,0, Math.PI/2));

    public static final int SIM_RES_WIDTH_PX = 1280;
    public static final int SIM_RES_HEIGHT_PX = 800;
    /** Placeholder until checked against the PhotonVision calibration. */
    public static final double SIM_DIAGONAL_FOV_DEG = 80.0;
    public static final double SIM_FPS = 30.0;
}
