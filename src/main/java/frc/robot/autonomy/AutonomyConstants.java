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

  /**
   * Where to seed the robot in SIM when it boots at the (0,0) corner (a blocked navgrid cell, so
   * pathfinding can't start there). A legal open pose in our zone; only used in simulation.
   */
  public static final Pose2d kSimStartPose = new Pose2d(2.0, 6.0, Rotation2d.fromDegrees(0));

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
  /** Time spent in the collection region roaming/gathering before going to shoot. */
  public static final double kCollectSeconds = 4.0;

  /**
   * Fire from anywhere within this radius of the shoot pose — we can shoot from anywhere on our
   * side (interp table), so this is a REGION, not a point. Generous so that diverting around an
   * opponent sitting on the nominal pose still counts as "in the shoot region" and we empty the
   * hopper ASAP.
   */
  public static final double kShootRegionRadiusMeters = 1.6;

  /** How far the collect ROAM ranges around a collection waypoint — cover the side, don't park. */
  public static final double kCollectRoamRadiusMeters = 1.5;

  /** "In the collection region" for the dwell — large, because we roam across it to gather. */
  public static final double kCollectRegionRadiusMeters = 2.5;

  /**
   * How long each roam waypoint is held before moving to the next (covers the area without thrash).
   */
  public static final double kRoamHoldSeconds = 1.5;

  /**
   * How long to keep FEEDING once the shot is live — long enough to empty the hopper (there is no
   * ball sensor, so we drain by time). The cycle stays in SHOOT until the feed gate has been open
   * (scoring SUCCEEDED) for this long. A robot/hopper property; bump it if the hopper holds more.
   */
  public static final double kShootEmptySeconds = 3.0;

  /**
   * Per-phase HARD cap (even if the pose can never be reached, or we can never aim) — guarantees
   * the cycle progresses. Must exceed travel + spin-up + {@link #kShootEmptySeconds} so a clean
   * shot gets its full empty window before this fires.
   */
  public static final double kPhaseHardTimeoutSeconds = 15.0;

  /** Half-period of the collection sweep oscillation. */
  public static final double kSweepHalfPeriodSeconds = 0.8;

  /** Match time (s) at/below which the endgame branch takes over. */
  public static final double kEndgameSeconds = 20.0;

  /**
   * Half-size of the keep-out box placed around each dynamic obstacle (opponent robot) before it is
   * handed to PathPlanner. PathPlanner pathfinds the robot as a POINT, so we inflate by opponent
   * half-footprint (~0.45 m) + our robot HALF-DIAGONAL (~0.59 m) so our center stays clear at any
   * heading.
   */
  public static final double kObstacleClearanceMeters = 1.05;
}
