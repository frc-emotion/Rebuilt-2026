package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autonomy.FieldGeometry.Rectangle;
import java.util.List;

/**
 * The hard navigation legality floor: a pure function that maps any requested target to the nearest
 * LEGAL target before it is ever handed to the navigator. It is belt-and-suspenders with the
 * navgrid (which the pathfinder uses to ROUTE) — the navgrid can be coarse or stale, but a target
 * that survives {@link #sanitize} is guaranteed on-field, outside every {@link FieldGeometry}
 * keep-out (hubs, towers, trenches — i.e. never "through the hub" and never UNDER a trench), and,
 * for shots, inside our own alliance zone (rule G407). Hardware-free and unit-tested.
 *
 * <p>It encodes the team-specific "no climb" decision implicitly: the tower footprints are
 * keep-outs, so the strategy can never aim the chassis into the tower base and earn a foul reaching
 * for a climb this robot cannot do.
 */
public final class LegalRegion {
  private LegalRegion() {}

  /** Bumper-to-structure clearance kept around every keep-out (robot ~0.84 m square). */
  public static final double kClearanceMeters = 0.45;

  /** Tiny nudge past an edge so the pushed-out point is strictly outside the inflated rectangle. */
  private static final double kEpsilonMeters = 0.02;

  private static final int kMaxPasses = 8;

  /** True if {@code p} is on the field and clear of every inflated keep-out. */
  public static boolean isLegal(Translation2d p) {
    return FieldGeometry.onField(p) && !FieldGeometry.inKeepout(p, kClearanceMeters);
  }

  /**
   * Return the nearest legal point to {@code target}: clamped onto the field, then pushed out of
   * any keep-out it lands in (smallest-penetration edge first, a few passes to settle overlaps near
   * the hub/trench band). If it somehow cannot be freed, fall back to a guaranteed-legal in-zone
   * anchor.
   */
  public static Translation2d sanitize(Translation2d target, Alliance alliance) {
    Translation2d p = clampToField(target);
    for (int pass = 0; pass < kMaxPasses; pass++) {
      if (!FieldGeometry.inKeepout(p, kClearanceMeters)) {
        return p;
      }
      p = clampToField(pushOutOfKeepouts(p));
    }
    return legalAnchor(alliance);
  }

  /**
   * Sanitize a SHOOT pose: force the translation into our alliance zone (G407), then run the normal
   * keep-out sanitize, preserving the requested heading. If forcing it into the zone is impossible,
   * fall back to the in-zone anchor facing the hub.
   */
  public static Pose2d sanitizeShoot(Pose2d shoot, Alliance alliance) {
    Translation2d t = clampIntoAllianceZone(shoot.getTranslation(), alliance);
    Translation2d legal = sanitize(t, alliance);
    if (!FieldGeometry.inAllianceZone(legal, alliance)) {
      legal = legalAnchor(alliance);
    }
    return new Pose2d(legal, shoot.getRotation());
  }

  /**
   * If {@code desired} is clear of every dynamic obstacle (boxes of half-size {@code clearance}),
   * return it unchanged. Otherwise search outward for the nearest LEGAL, obstacle-free pose
   * (keeping the requested heading) so we drive to an OPEN spot in the region instead of a point
   * buried inside an opponent — PathPlanner cannot path to a target inside an obstacle. Falls back
   * to the desired pose if nothing nearby is open.
   */
  public static Pose2d nearestClear(
      Pose2d desired, List<Translation2d> obstacles, double clearance) {
    if (obstacles.isEmpty() || clearOf(desired.getTranslation(), obstacles, clearance)) {
      return desired;
    }
    double[] radii = {clearance + 0.3, clearance + 0.7, clearance + 1.1, clearance + 1.5};
    for (double r : radii) {
      for (int deg = 0; deg < 360; deg += 30) {
        double a = Math.toRadians(deg);
        Translation2d cand =
            desired.getTranslation().plus(new Translation2d(r * Math.cos(a), r * Math.sin(a)));
        if (isLegal(cand) && clearOf(cand, obstacles, clearance)) {
          return new Pose2d(cand, desired.getRotation());
        }
      }
    }
    return desired;
  }

  /** True if {@code p} is outside every obstacle's keep-out box. */
  public static boolean clearOf(Translation2d p, List<Translation2d> obstacles, double clearance) {
    for (Translation2d o : obstacles) {
      if (Math.abs(o.getX() - p.getX()) < clearance && Math.abs(o.getY() - p.getY()) < clearance) {
        return false;
      }
    }
    return true;
  }

  // ── internals ──

  private static Translation2d clampToField(Translation2d p) {
    double x =
        clamp(p.getX(), kClearanceMeters, FieldGeometry.kFieldLengthMeters - kClearanceMeters);
    double y =
        clamp(p.getY(), kClearanceMeters, FieldGeometry.kFieldWidthMeters - kClearanceMeters);
    return new Translation2d(x, y);
  }

  private static Translation2d clampIntoAllianceZone(Translation2d p, Alliance alliance) {
    double edge = kClearanceMeters;
    if (alliance == Alliance.Red) {
      double minX =
          FieldGeometry.kFieldLengthMeters - FieldGeometry.kAllianceZoneDepthMeters + edge;
      return new Translation2d(Math.max(p.getX(), minX), p.getY());
    }
    double maxX = FieldGeometry.kAllianceZoneDepthMeters - edge;
    return new Translation2d(Math.min(p.getX(), maxX), p.getY());
  }

  // Push p out of the first keep-out that contains it, along the axis of least penetration.
  private static Translation2d pushOutOfKeepouts(Translation2d p) {
    for (Rectangle r : FieldGeometry.kKeepouts) {
      Rectangle e = r.expandedBy(kClearanceMeters);
      if (!e.contains(p)) {
        continue;
      }
      double left = p.getX() - e.minX(); // move -X to exit
      double right = e.maxX() - p.getX(); // move +X to exit
      double down = p.getY() - e.minY(); // move -Y to exit
      double up = e.maxY() - p.getY(); // move +Y to exit
      double min = Math.min(Math.min(left, right), Math.min(down, up));
      if (min == left) {
        return new Translation2d(e.minX() - kEpsilonMeters, p.getY());
      } else if (min == right) {
        return new Translation2d(e.maxX() + kEpsilonMeters, p.getY());
      } else if (min == down) {
        return new Translation2d(p.getX(), e.minY() - kEpsilonMeters);
      }
      return new Translation2d(p.getX(), e.maxY() + kEpsilonMeters);
    }
    return p;
  }

  /**
   * A guaranteed-legal point in our alliance zone, in front of and facing nothing in particular.
   */
  private static Translation2d legalAnchor(Alliance alliance) {
    double hubY = FieldGeometry.hubCenter(alliance).getY();
    double x = alliance == Alliance.Red ? FieldGeometry.kFieldLengthMeters - 2.0 : 2.0;
    return new Translation2d(x, hubY);
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
