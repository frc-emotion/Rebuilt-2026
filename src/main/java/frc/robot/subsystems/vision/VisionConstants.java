package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Map;

/**
 * Vision constants, copied verbatim from legacy VisionConstants except MAX_POSE_AMBIGUITY
 * (re-enabled at 0.3 per team decision D5; legacy had it disabled at 1.0 with tuned 0.15 commented
 * out). Pose-estimation constants are NEW (D11).
 *
 * <p>Coordinate System (WPILib / PhotonVision convention for Transform3d): X = forward (out of
 * camera lens / out of tag face) Y = left Z = up
 */
public final class VisionConstants {
  private VisionConstants() {}

  /** When true, tracks ANY visible AprilTag with raw camera→tag data (no hub offset). */
  public static final boolean BENCH_TEST_ANY_TAG = false;

  /** Which camera name is physically on the turret. Change to match your wiring. */
  public static final String TURRET_CAM_NAME = "mugilanr"; // "aaranc"

  /** Reject any single-tag reading with ambiguity above this (0 = perfect, 1 = garbage). */
  public static final double MAX_POSE_AMBIGUITY = 0.3; // team decision D5 (legacy: 1.0 //0.15)

  // ================================================================
  //  POSE ESTIMATION (new, D11) — camera is TURRET-MOUNTED
  // ================================================================

  // TODO(measure): turret azimuth axis in the robot frame (x fwd, y left, z up from floor).
  // Placeholder until the team measures; pose estimation refuses to run while either
  // transform is the unmeasured placeholder (kTransformsMeasured flag below).
  public static final Transform3d ROBOT_TO_TURRET =
      new Transform3d(new Translation3d(0.0, 0.0, 0.5), new Rotation3d());

  // TODO(measure): camera in the turret frame WITH THE TURRET AT ZERO (boot straight-forward).
  public static final Transform3d TURRET_TO_CAMERA =
      new Transform3d(new Translation3d(0.2, 0.0, 0.2), new Rotation3d());

  // Flip to true once ROBOT_TO_TURRET / TURRET_TO_CAMERA hold real measurements.
  public static final boolean kTransformsMeasured = false;

  // Skip pose frames while the turret slews faster than this — "current turret angle" is then a
  // valid stand-in for the angle at capture time, so no angle-history buffering is needed.
  public static final double kMaxTurretSlewForPoseRps = 0.25;

  // Standard photonlib two-tier std devs, scaled by average tag distance in the estimator.
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  // ================================================================
  //  HUB GEOMETRY (verbatim)
  // ================================================================

  /** Hub is a 47" × 47" rectangular prism. Half-width = depth from any face to center. */
  private static final double HUB_DEPTH_METERS = 0.604; // derived from FRC2026_WELDED.json

  private static final double HUB_LATERAL_OFFSET_METERS = 0.356; // offset tags, ~14"

  public static final Translation2d RED_HUB_CENTER = new Translation2d(11.916, 4.035);
  public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.626, 4.035);

  /** Hub funnel top height from floor */
  public static final double HUB_SCORING_HEIGHT_METERS = Units.inchesToMeters(72.0);

  /** Hub AprilTag center height from floor */
  public static final double HUB_TAG_HEIGHT_METERS = 1.12395; // ~44.25"

  // ================================================================
  //  HUB TAG IDS (verbatim)
  // ================================================================

  public static final int[] RED_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};
  public static final int[] BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};

  // Tags physically located in each zone
  public static final int[] RED_ZONE_TAG_IDS = {7, 9, 10, 12};
  public static final int[] BLUE_ZONE_TAG_IDS = {23, 25, 26, 28};
  public static final int[] NEUTRAL_RED_SIDE_TAG_IDS = {1, 3, 4, 6};
  public static final int[] NEUTRAL_BLUE_SIDE_TAG_IDS = {17, 19, 20, 22};

  public static boolean isRedHubTag(int tagId) {
    return contains(RED_HUB_TAG_IDS, tagId);
  }

  public static boolean isBlueHubTag(int tagId) {
    return contains(BLUE_HUB_TAG_IDS, tagId);
  }

  public static boolean isHubTag(int tagId) {
    return isRedHubTag(tagId) || isBlueHubTag(tagId);
  }

  /** Returns true if the tag belongs to OUR alliance's hub. */
  public static boolean isOurHubTag(int tagId) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return (alliance == Alliance.Red) ? isRedHubTag(tagId) : isBlueHubTag(tagId);
  }

  public static boolean isRedZonePassingTag(int tagId) {
    return contains(RED_ZONE_TAG_IDS, tagId);
  }

  public static boolean isBlueZonePassingTag(int tagId) {
    return contains(BLUE_ZONE_TAG_IDS, tagId);
  }

  public static boolean isNeutralRedSidePassingTag(int tagId) {
    return contains(NEUTRAL_RED_SIDE_TAG_IDS, tagId);
  }

  public static boolean isNeutralBlueSidePassingTag(int tagId) {
    return contains(NEUTRAL_BLUE_SIDE_TAG_IDS, tagId);
  }

  public static boolean isPassingTag(int tagId) {
    return isRedZonePassingTag(tagId)
        || isBlueZonePassingTag(tagId)
        || isNeutralRedSidePassingTag(tagId)
        || isNeutralBlueSidePassingTag(tagId);
  }

  /**
   * Returns true if the tag is one we'd pass to when on our alliance. (We pass into the opponent's
   * zone and use the opponent-side neutral tags.)
   */
  public static boolean isOurPassingTag(int tagId) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Red) {
      return isBlueZonePassingTag(tagId) || isNeutralRedSidePassingTag(tagId);
    }
    return isRedZonePassingTag(tagId) || isNeutralBlueSidePassingTag(tagId);
  }

  private static boolean contains(int[] ids, int tagId) {
    for (int id : ids) {
      if (id == tagId) {
        return true;
      }
    }
    return false;
  }

  // ================================================================
  //  TAG → HUB CENTER TRANSFORMS (verbatim — the -0.2 m lateral fudge is hand-tuned and
  //  team-confirmed correct, D6. Do not "fix" it.)
  // ================================================================
  //
  // Each Transform3d goes from the TAG's coordinate frame to the HUB CENTER.
  //   X = -HUB_DEPTH  (hub center is BEHIND the tag face)
  //   Y = lateral offset in tag-local frame (0 for centered tags)
  //   Z = 0 (we ignore vertical offset for ground distance)

  private static Transform3d hubVec(double lateralY) {
    return new Transform3d(new Translation3d(-HUB_DEPTH_METERS, lateralY, 0.0), new Rotation3d());
  }

  /** Tag ID → Transform3d from tag frame to hub center. Only hub tags have entries. */
  public static final Map<Integer, Transform3d> TAG_TO_HUB_CENTER =
      Map.ofEntries(
          // ── Red Hub ──
          Map.entry(2, hubVec(0.000 - 0.2)), // +Y face, centered
          Map.entry(3, hubVec(+HUB_LATERAL_OFFSET_METERS - 0.2)), // -X face, offset
          Map.entry(4, hubVec(0.000 - 0.2)), // -X face, centered
          Map.entry(5, hubVec(0.000 - 0.2)), // -Y face, centered
          Map.entry(8, hubVec(+HUB_LATERAL_OFFSET_METERS - 0.2)), // -Y face, offset
          Map.entry(9, hubVec(+HUB_LATERAL_OFFSET_METERS - 0.2)), // +X face, offset
          Map.entry(10, hubVec(0.000 - 0.2)), // +X face, centered
          Map.entry(11, hubVec(+HUB_LATERAL_OFFSET_METERS - 0.2)), // +Y face, offset

          // ── Blue Hub ──
          Map.entry(18, hubVec(0.000 - 0.2)), // -Y face, centered
          Map.entry(19, hubVec(+HUB_LATERAL_OFFSET_METERS - 0.2)), // +X face, offset
          Map.entry(20, hubVec(0.000 - 0.2)), // +X face, centered
          Map.entry(21, hubVec(0.000 - 0.2)), // +Y face, centered
          Map.entry(24, hubVec(+HUB_LATERAL_OFFSET_METERS - 0.2)), // +Y face, offset
          Map.entry(25, hubVec(+HUB_LATERAL_OFFSET_METERS - 0.2)), // -X face, offset
          Map.entry(26, hubVec(0.000 - 0.2)), // -X face, centered
          Map.entry(
              27, hubVec(+HUB_LATERAL_OFFSET_METERS - 0.2)) // -Y face, offset WAS POSITIVE BEFORE
          );
}
