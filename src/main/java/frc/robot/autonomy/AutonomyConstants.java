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
  // Blue hub center is (4.626, 4.035) with hub tags 18-27 around it (VisionConstants). The shoot
  // pose sits ~3 m in front of the hub FACING it, so the turret camera can see a hub tag and the
  // feed gate can open — without line-of-sight to a tag the shot deliberately never fires (W12).
  /** A safe pose on our side to shoot from, facing the blue hub. */
  public static final Pose2d kShootPose = new Pose2d(1.65, 4.035, Rotation2d.fromDegrees(0));

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

  /** How close (meters) counts as "arrived" at a pose (the navigator's path tolerance). */
  public static final double kArrivalToleranceMeters = 0.15;

  /**
   * How close (meters) to a phase's nominal pose counts as "parked in the region" for the dwell —
   * loose enough that the collection sweep oscillating within the region still counts as parked.
   */
  public static final double kDwellRegionToleranceMeters = 0.6;

  // ── Cycle dwell + debounce (anti-thrash). Dwells are ARRIVAL-based — they count only while the
  //    robot is parked at the phase's pose, so travel time never eats the window (see MatchCycle).
  // ──
  /** Minimum time PARKED at the collection region before going to shoot (the gathering dwell). */
  public static final double kCollectSeconds = 2.0;

  /** Minimum time PARKED at the shoot pose before a confirmed shot may end the phase. */
  public static final double kMinShootSeconds = 1.0;

  /** Max time PARKED at the shoot pose firing before giving up this shot and going to collect. */
  public static final double kShootMaxSeconds = 4.0;

  /**
   * Per-phase HARD cap (even if the pose can never be reached) — guarantees the cycle progresses.
   */
  public static final double kPhaseHardTimeoutSeconds = 12.0;

  /** Half-period of the collection sweep oscillation. */
  public static final double kSweepHalfPeriodSeconds = 0.8;

  /** Match time (s) at/below which the endgame branch takes over. */
  public static final double kEndgameSeconds = 20.0;
}
