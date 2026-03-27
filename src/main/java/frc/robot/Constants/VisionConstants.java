package frc.robot.Constants;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the vision subsystem.
 * Contains camera configurations, transforms, and tuning parameters.
 * 
 * Coordinate System (WPILib/PhotonVision convention):
 * - X: Forward (positive = front of robot)
 * - Y: Left (positive = left side of robot)
 * - Z: Up (positive = above robot)
 * - Pitch: Rotation around Y axis (positive = tilt up)
 * - Yaw: Rotation around Z axis (positive = rotate left/CCW)
 * - Roll: Rotation around X axis
 */
public final class VisionConstants {

    public static final boolean ENABLE_TURRET_CAM = true;

    /** When true, turret auto-aim tracks ANY visible AprilTag (not just hub tags).
     *  Set to false for competition so the turret only locks onto the hub. */
    public static final boolean BENCH_TEST_ANY_TAG = false;

    /** Which camera name is physically on the turret. Change to match your wiring. */
    public static final String TURRET_CAM_NAME = "mugilanr"; // "aaranc"

    // ==================
    // FIELD LAYOUT
    // ==================
    // Load the 2026 Rebuilt AprilTag field layout from deploy directory

    public static final AprilTagFieldLayout TAG_LAYOUT;
    static {
        AprilTagFieldLayout layout = null;
        try {
            System.out.println("Loading custom field layout from deploy directory...");
            System.out.println("Deploy directory: " + Filesystem.getDeployDirectory());

            layout = new AprilTagFieldLayout(Path.of(Filesystem.getDeployDirectory() + "/FRC2026_WELDED.json"));
        } catch (IOException e) {
            System.err.println("Failed to load custom field layout");
            e.printStackTrace();

        }
        TAG_LAYOUT = layout;
    }

    // ==================
    // TURRET CAMERA CONFIGURATION
    // ==================

    /**
     * Static transform from robot center to turret pivot point.
     * This is where the turret rotation axis is located.
     * 
     * TODO: Update these values to match your robot's turret mounting position
     * Position: X (forward), Y (left), Z (up) from robot center
     */
    public static final Transform3d ROBOT_TO_TURRET_PIVOT = new Transform3d(
            new Translation3d(
                    Units.inchesToMeters(-5.5),   // X: turret pivot behind robot center (back bumper side)
                    Units.inchesToMeters(6.5),    // Y: turret pivot left of robot center
                    Units.inchesToMeters(18.064)  // Z: turret pivot height above floor
            ),
            new Rotation3d(0, 0, Units.degreesToRadians(180)) // Turret faces backward (-X) at position 0
    );

    /**
     * Static transform from turret pivot to camera.
     * This is the camera's position relative to the turret's rotation point.
     * This transform rotates WITH the turret.
     * 
     * TODO: Update these values to match your camera mounting on the turret
     */
    public static final Transform3d TURRET_PIVOT_TO_CAM = new Transform3d(
            new Translation3d(
                    Units.inchesToMeters(0.0),    // X: camera is inline with pivot axis
                    Units.inchesToMeters(5.25),   // Y: camera is 5.25" left of turret centerline
                    Units.inchesToMeters(8.0)     // Z: camera is 8" above pivot point
            ),
            new Rotation3d(
                    0, // Roll: 0°
                    Units.degreesToRadians(10), // Pitch: +10° = nose UP (confirmed: getPitch is negative for above-camera tags)
                    0 // Yaw: 0° (points forward along turret)
            ));

    /** Height of the turret camera lens from the floor in meters (measured on robot) */
    public static final double CAM_TURRET_HEIGHT_METERS = Units.inchesToMeters(26.5);

    // ==================
    // ★★★ TURRET CAMERA DISTANCE CALIBRATION ★★★
    // ==================
    // These 2 constants control the trig distance formula.
    // If distance reads wrong, adjust TURRET_CAM_TILT_DEG.
    //
    // QUICK CALIBRATION (5 min in pit):
    //   1. Place robot so bumper is exactly 2m from the AprilTag (tape measure).
    //   2. Aim turret at the tag. Read "targetPitchDeg" from Elastic telemetry.
    //   3. Compute:  tilt = targetPitchDeg - degrees(atan(0.451 / (2.0 - 0.356)))
    //              = targetPitchDeg - 15.3
    //   4. Put that number here and redeploy.
    //
    // If distance is too HIGH → make this value MORE negative.
    // If distance is too LOW  → make this value LESS negative.
    // If distance is 0        → this value is not negative enough (or no tag visible).

    /**
     * Camera tilt in degrees. Negative = camera tilted UP from horizontal.
     * Your camera is aimed ABOVE the horizon, so this is negative.
     * This is the single most important constant for distance accuracy.
     */
    public static final double TURRET_CAM_TILT_DEG = -10.0;

    /**
     * Horizontal offset from turret camera to front bumper along the shooting axis.
     * Added to the camera's horizontal distance to approximate bumper-to-hub distance
     * (which is how the interpolation table was calibrated).
     * Measure on robot: distance from camera lens to front bumper edge.
     */
    public static final double CAMERA_TO_BUMPER_OFFSET_METERS = Units.inchesToMeters(14.0);

    // ==================
    // HUB TARGETING CONFIGURATION
    // ==================
    // The turret aims at the hub center, NOT the AprilTag center.
    // This automatically handles position-based offset for scoring.

    /**
     * Red alliance hub center position (field coordinates in meters).
     * Calculated from average of hub AprilTag positions in FRC2026_WELDED.json
     * Tags 2-5 and 8-11 surround the red hub.
     */
    public static final Translation2d RED_HUB_CENTER = new Translation2d(
            11.92, // X: average of hub tag X positions
            4.035 // Y: average of hub tag Y positions (field center)
    );

    /**
     * Blue alliance hub center position (field coordinates in meters).
     * Calculated from average of hub AprilTag positions in FRC2026_WELDED.json
     * Tags 18-21 and 24-27 surround the blue hub.
     */
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(
            4.63, // X: average of hub tag X positions
            4.035 // Y: average of hub tag Y positions (field center)
    );

    /** Hub funnel top height from floor (for trajectory calculations) */
    public static final double HUB_SCORING_HEIGHT_METERS = Units.inchesToMeters(72.0);

    /** Hub AprilTag height from floor */
    public static final double HUB_TAG_HEIGHT_METERS = 1.12395; // ~44.25 inches

    /**
     * AprilTag IDs mounted on the Red alliance hub.
     * These are used for turret targeting validation.
     */
    public static final int[] RED_HUB_TAG_IDS = { 2, 3, 4, 5, 8, 9, 10, 11 };

    /**
     * AprilTag IDs mounted on the Blue alliance hub.
     * These are used for turret targeting validation.
     */
    public static final int[] BLUE_HUB_TAG_IDS = { 18, 19, 20, 21, 24, 25, 26, 27 };

    /**
     * DRIVER-STATION-FACING hub tags only. These are the tags visible when
     * shooting from your scoring zone. Using only these avoids locking onto
     * tags on the far side of the hub which give bad tx readings.
     *
     * PLACEHOLDER — verify these IDs on the physical field.
     * Analysis from FRC2026_WELDED.json quaternion orientations:
     *   Red tags 9,10 face +X (toward red DS at x≈16.5)
     *   Blue tags 25,26 face -X (toward blue DS at x≈0)
     */
    public static final int[] RED_DS_FACING_TAG_IDS = { 9, 10 };
    public static final int[] BLUE_DS_FACING_TAG_IDS = { 25, 26 };

    /**
     * Checks if a tag ID belongs to the red hub.
     */
    public static boolean isRedHubTag(int tagId) {
        for (int id : RED_HUB_TAG_IDS) {
            if (id == tagId)
                return true;
        }
        return false;
    }

    /**
     * Checks if a tag ID belongs to the blue hub.
     */
    public static boolean isBlueHubTag(int tagId) {
        for (int id : BLUE_HUB_TAG_IDS) {
            if (id == tagId)
                return true;
        }
        return false;
    }

    /**
     * Checks if a tag ID belongs to any hub.
     */
    public static boolean isHubTag(int tagId) {
        return isRedHubTag(tagId) || isBlueHubTag(tagId);
    }

    /**
     * Checks if a tag ID is a driver-station-facing hub tag (either alliance).
     * Use this for turret tracking to avoid locking onto far-side tags.
     */
    public static boolean isDSFacingHubTag(int tagId) {
        for (int id : RED_DS_FACING_TAG_IDS) {
            if (id == tagId) return true;
        }
        for (int id : BLUE_DS_FACING_TAG_IDS) {
            if (id == tagId) return true;
        }
        return false;
    }

    // ==================
    // STANDARD DEVIATIONS
    // ==================
    // These control how much we trust vision vs odometry
    // Lower values = more trust in that measurement
    // Format: [x, y, theta] in meters and radians

    /** Single tag standard deviations (less accurate, higher = less trust) */
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.5, 1.5, 3);

    /** Multi-tag standard deviations (more accurate due to multiple references) */
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 0.5);

    // ==================
    // VISION TUNING PARAMETERS
    // ==================

    /** Maximum distance (meters) at which we trust vision measurements */
    public static final double MAX_VISION_DISTANCE = 7.0;

    /** Maximum pose ambiguity we'll accept (0-1, lower is better) */
    public static final double MAX_AMBIGUITY = 0.2;

    /** Minimum number of targets for multi-tag to be considered reliable */
    public static final int MIN_MULTI_TAG_TARGETS = 2;

    private VisionConstants() {}
}
