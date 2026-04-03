package frc.robot.Constants;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Constants for the vision subsystem.
 *
 * Coordinate System (WPILib / PhotonVision convention for Transform3d):
 *   X = forward (out of camera lens / out of tag face)
 *   Y = left
 *   Z = up
 */
public final class VisionConstants {

    public static final boolean ENABLE_TURRET_CAM = true;

    /** When true, tracks ANY visible AprilTag with raw camera→tag data (no hub offset).
     *  When false, only tracks our alliance hub tags with camera→tag→hub math. */
    public static final boolean BENCH_TEST_ANY_TAG = false;

    /** Which camera name is physically on the turret. Change to match your wiring. */
    public static final String TURRET_CAM_NAME = "mugilanr"; // "aaranc"

    /** Reject any single-tag reading with ambiguity above this (0 = perfect, 1 = garbage). */
    public static final double MAX_POSE_AMBIGUITY = 1.0;//0.15;

    // ================================================================
    //  HUB GEOMETRY
    // ================================================================

    /** Hub is a 47" × 47" rectangular prism. Half-width = depth from any face to center. */
    private static final double HUB_DEPTH_METERS = 0.604; // derived from FRC2026_WELDED.json
    private static final double HUB_LATERAL_OFFSET_METERS = 0.356; // offset tags, ~14"

    /**
     * Red alliance hub center (field coordinates, meters).
     * Derived from midpoint of opposite-face tag positions in FRC2026_WELDED.json.
     */
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.916, 4.035);

    /**
     * Blue alliance hub center (field coordinates, meters).
     */
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.626, 4.035);

    /** Hub funnel top height from floor */
    public static final double HUB_SCORING_HEIGHT_METERS = Units.inchesToMeters(72.0);

    /** Hub AprilTag center height from floor */
    public static final double HUB_TAG_HEIGHT_METERS = 1.12395; // ~44.25"

    // ================================================================
    //  HUB TAG IDS
    // ================================================================

    public static final int[] RED_HUB_TAG_IDS  = { 2, 3, 4, 5, 8, 9, 10, 11 };
    public static final int[] BLUE_HUB_TAG_IDS = { 18, 19, 20, 21, 24, 25, 26, 27 };

    public static boolean isRedHubTag(int tagId) {
        for (int id : RED_HUB_TAG_IDS) { if (id == tagId) return true; }
        return false;
    }

    public static boolean isBlueHubTag(int tagId) {
        for (int id : BLUE_HUB_TAG_IDS) { if (id == tagId) return true; }
        return false;
    }

    public static boolean isHubTag(int tagId) {
        return isRedHubTag(tagId) || isBlueHubTag(tagId);
    }

    /** Returns true if the tag belongs to OUR alliance's hub. */
    public static boolean isOurHubTag(int tagId) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return (alliance == Alliance.Red) ? isRedHubTag(tagId) : isBlueHubTag(tagId);
    }

    // ================================================================
    //  TAG → HUB CENTER TRANSFORMS
    // ================================================================
    //
    // Each Transform3d goes from the TAG's coordinate frame to the HUB CENTER.
    //   X = -HUB_DEPTH  (hub center is BEHIND the tag face)
    //   Y = lateral offset in tag-local frame (0 for centered tags)
    //   Z = 0 (we ignore vertical offset for ground distance)
    //
    // Computed from FRC2026_WELDED.json tag positions + hub center.
    // TODO: verify lateral offsets from 2026 Field Dimension Drawings.

    private static Transform3d hubVec(double lateralY) {
        return new Transform3d(
                new Translation3d(-HUB_DEPTH_METERS, lateralY, 0.0),
                new Rotation3d());
    }

    /** Tag ID → Transform3d from tag frame to hub center. Only hub tags have entries. */
    public static final Map<Integer, Transform3d> TAG_TO_HUB_CENTER = Map.ofEntries(
            // ── Red Hub ──
            Map.entry( 2, hubVec( 0.000-0.2)),            // +Y face, centered
            Map.entry( 3, hubVec(+HUB_LATERAL_OFFSET_METERS-0.2)), // -X face, offset
            Map.entry( 4, hubVec(0.000-0.2)),            // -X face, centered
            Map.entry( 5, hubVec( 0.000-0.2)),            // -Y face, centered
            Map.entry( 8, hubVec(+HUB_LATERAL_OFFSET_METERS-0.2)), // -Y face, offset
            Map.entry( 9, hubVec(+HUB_LATERAL_OFFSET_METERS-0.2)), // +X face, offset
            Map.entry(10, hubVec( 0.000-0.2)),            // +X face, centered
            Map.entry(11, hubVec(+HUB_LATERAL_OFFSET_METERS-0.2)), // +Y face, offset

            // ── Blue Hub ──
            Map.entry(18, hubVec( 0.000-0.2)),            // -Y face, centered
            Map.entry(19, hubVec(+HUB_LATERAL_OFFSET_METERS-0.2)), // +X face, offset
            Map.entry(20, hubVec( 0.000-0.2)),            // +X face, centered
            Map.entry(21, hubVec( 0.000-0.2)),            // +Y face, centered
            Map.entry(24, hubVec(+HUB_LATERAL_OFFSET_METERS-0.2)), // +Y face, offset
            Map.entry(25, hubVec(+HUB_LATERAL_OFFSET_METERS-0.2)), // -X face, offset
            Map.entry(26, hubVec( 0.000-0.2)),            // -X face, centered
            Map.entry(27, hubVec(+HUB_LATERAL_OFFSET_METERS-0.2))  // -Y face, offset WAS POSITIVE BEFORE
    );

    private VisionConstants() {}
}
