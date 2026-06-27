package frc.robot.autonomy;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Field geometry + timing for the v1 match autonomy. ALL field poses are PLACEHOLDERS in
 * blue-origin meters (2027 game geometry is unknown); they are seeded from real anchors in the
 * current deploy paths so the sim actually drives between two distinct points. They are config
 * constants on purpose — when the real field is known, only this file changes. Poses are
 * blue-relative; the navigator alliance-flips them.
 */
public final class AutonomyConstants {
  private AutonomyConstants() {}

  // ── Field poses (PLACEHOLDER — blue origin, meters) ──
  /** A safe pose on our side to shoot from (we can score from anywhere on our side). */
  public static final Pose2d kShootPose = new Pose2d(3.58, 0.75, Rotation2d.fromDegrees(0));

  /** The collection region (near the Depot anchor in the current paths). */
  public static final Pose2d kCollectionPose = new Pose2d(0.72, 5.9, Rotation2d.fromDegrees(0));

  /** Lateral offset the collection sweep oscillates across to gather pieces (no piece tracking). */
  public static final double kSweepLateralMeters = 0.6;

  /** Endgame park pose (PLACEHOLDER; this robot has no climb — see docs/autonomy.md). */
  public static final Pose2d kEndgameParkPose = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(0));

  // ── Pathfinding constraints (from deploy/pathplanner/settings.json; clamp to the PathPlanner
  //    5.44 m/s, NOT TunerConstants' 5.85 — the known settings-vs-tuner mismatch). ──
  public static final PathConstraints kPathConstraints =
      new PathConstraints(
          3.0, 3.0, Units.degreesToRadians(540.0), Units.degreesToRadians(720.0), 12.0);

  /** How close (meters) counts as "arrived" at a pose. */
  public static final double kArrivalToleranceMeters = 0.15;

  // ── Cycle dwell + debounce (anti-thrash) ──
  /** Minimum time gathering at the collection region before going to shoot (the dwell). */
  public static final double kCollectSeconds = 2.0;

  /** Minimum time committed to the shoot phase once there (prevents instant bail). */
  public static final double kMinShootSeconds = 1.0;

  /** Hard cap on the shoot phase so a never-confirming shot can't hang the cycle. */
  public static final double kShootTimeoutSeconds = 3.0;

  /** Half-period of the collection sweep oscillation. */
  public static final double kSweepHalfPeriodSeconds = 0.8;

  /** Match time (s) at/below which the endgame branch takes over. */
  public static final double kEndgameSeconds = 20.0;
}
