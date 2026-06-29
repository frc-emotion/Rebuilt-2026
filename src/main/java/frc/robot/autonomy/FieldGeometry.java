package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.List;

/**
 * The 2026 "REBUILT" field geometry as data — the single source of truth for navigation boundaries.
 * Everything here is BLUE-ORIGIN meters (WPILib convention: +X toward red, +Y toward the far
 * sideline); the navigator alliance-flips targets, and {@link #inAllianceZone} takes the alliance
 * explicitly. Both {@link LegalRegion} (the runtime target sanitizer) and the navgrid generator
 * read these same constants, so the keep-outs the planner sees and the keep-outs the sanitizer
 * enforces can never drift apart.
 *
 * <p>Hub centers are reused VERBATIM from {@link VisionConstants} (tuned numbers are sacred — never
 * duplicate-and-drift). Structure footprints are derived from the published 2026 dimensions + the
 * deployed AprilTag layout ({@code FRC2026_WELDED.json}). The HUB and TOWER footprints are
 * well-anchored (tag-derived); the TRENCH footprints are marked APPROXIMATE — they exist to force
 * the pathfinder to cross via the BUMPS (drive-over ramps, left open) instead of routing a too-tall
 * robot UNDER a TRENCH (~0.565 m clearance). Refine the TRENCH rectangles against the real field
 * CAD when available; only this file changes.
 */
public final class FieldGeometry {
  private FieldGeometry() {}

  // ── Field extents (FRC2026_WELDED.json field object) ──
  public static final double kFieldLengthMeters = 16.541;
  public static final double kFieldWidthMeters = 8.069;
  public static final double kCenterLineX = kFieldLengthMeters / 2.0;

  // ── Hub (reused from VisionConstants — DO NOT duplicate the tuned centers) ──
  public static final Translation2d kBlueHubCenter = VisionConstants.BLUE_HUB_CENTER;
  public static final Translation2d kRedHubCenter = VisionConstants.RED_HUB_CENTER;

  /** Hub is a 47" × 47" prism; half-extent from center to any face. */
  private static final double kHubHalfMeters = Units.inchesToMeters(47.0) / 2.0; // ~0.597

  // ── Alliance zone (158.6" deep, full field width; bounded by the alliance wall) ──
  public static final double kAllianceZoneDepthMeters = Units.inchesToMeters(158.6); // ~4.029

  // ── Tower footprint (49.25"W × 45"D, in the alliance wall between DS2/DS3) ──
  // Y-center taken from the tower-wall AprilTags (blue 31/32, red 15/16); depth extends from the
  // wall into the field.
  private static final double kTowerWidthMeters = Units.inchesToMeters(49.25); // ~1.251 (spans Y)
  private static final double kTowerDepthMeters = Units.inchesToMeters(45.0); // ~1.143 (spans X)
  private static final double kBlueTowerYCenter = 3.9616; // (tags 31,32: 3.7457 & 4.1775)
  private static final double kRedTowerYCenter = 4.1077; // (tags 15,16: 4.3236 & 3.8918)

  // ── Trench footprint (65.65"W × 47"D) — APPROXIMATE placement (see class doc) ──
  private static final double kTrenchWidthMeters = Units.inchesToMeters(65.65); // ~1.668 (spans Y)
  // Trenches sit at the alliance/neutral boundary band, flanking the BUMP+HUB toward each sideline.
  private static final double kTrenchBandHalfDepth = kHubHalfMeters; // share the hub's X depth band

  /** An axis-aligned rectangle in field meters. */
  public record Rectangle(double minX, double minY, double maxX, double maxY) {
    public boolean contains(Translation2d p) {
      return p.getX() >= minX && p.getX() <= maxX && p.getY() >= minY && p.getY() <= maxY;
    }

    /** Same rectangle grown by {@code m} on every side (robot-radius inflation, etc.). */
    public Rectangle expandedBy(double m) {
      return new Rectangle(minX - m, minY - m, maxX + m, maxY + m);
    }
  }

  private static Rectangle centered(Translation2d c, double halfX, double halfY) {
    return new Rectangle(c.getX() - halfX, c.getY() - halfY, c.getX() + halfX, c.getY() + halfY);
  }

  // ── Keep-outs (robots may not drive here) ──
  public static final Rectangle kBlueHub = centered(kBlueHubCenter, kHubHalfMeters, kHubHalfMeters);
  public static final Rectangle kRedHub = centered(kRedHubCenter, kHubHalfMeters, kHubHalfMeters);

  public static final Rectangle kBlueTower =
      new Rectangle(
          0.0,
          kBlueTowerYCenter - kTowerWidthMeters / 2.0,
          kTowerDepthMeters,
          kBlueTowerYCenter + kTowerWidthMeters / 2.0);
  public static final Rectangle kRedTower =
      new Rectangle(
          kFieldLengthMeters - kTowerDepthMeters,
          kRedTowerYCenter - kTowerWidthMeters / 2.0,
          kFieldLengthMeters,
          kRedTowerYCenter + kTowerWidthMeters / 2.0);

  // Blue trenches: boundary band at the blue hub's X depth, near each sideline. APPROXIMATE.
  public static final Rectangle kBlueTrenchNear =
      new Rectangle(
          kBlueHubCenter.getX() - kTrenchBandHalfDepth,
          0.0,
          kBlueHubCenter.getX() + kTrenchBandHalfDepth,
          kTrenchWidthMeters);
  public static final Rectangle kBlueTrenchFar =
      new Rectangle(
          kBlueHubCenter.getX() - kTrenchBandHalfDepth,
          kFieldWidthMeters - kTrenchWidthMeters,
          kBlueHubCenter.getX() + kTrenchBandHalfDepth,
          kFieldWidthMeters);
  public static final Rectangle kRedTrenchNear =
      new Rectangle(
          kRedHubCenter.getX() - kTrenchBandHalfDepth,
          0.0,
          kRedHubCenter.getX() + kTrenchBandHalfDepth,
          kTrenchWidthMeters);
  public static final Rectangle kRedTrenchFar =
      new Rectangle(
          kRedHubCenter.getX() - kTrenchBandHalfDepth,
          kFieldWidthMeters - kTrenchWidthMeters,
          kRedHubCenter.getX() + kTrenchBandHalfDepth,
          kFieldWidthMeters);

  /** Every hard keep-out (both alliances' structures — the field is shared). */
  public static final List<Rectangle> kKeepouts =
      List.of(
          kBlueHub,
          kRedHub,
          kBlueTower,
          kRedTower,
          kBlueTrenchNear,
          kBlueTrenchFar,
          kRedTrenchNear,
          kRedTrenchFar);

  // ── Queries ──

  public static Translation2d hubCenter(Alliance alliance) {
    return alliance == Alliance.Red ? kRedHubCenter : kBlueHubCenter;
  }

  /** True if {@code p} is inside the given alliance's scoring zone (G407: shoot only from here). */
  public static boolean inAllianceZone(Translation2d p, Alliance alliance) {
    if (alliance == Alliance.Red) {
      return p.getX() >= kFieldLengthMeters - kAllianceZoneDepthMeters;
    }
    return p.getX() <= kAllianceZoneDepthMeters;
  }

  /**
   * True if {@code p} is inside any hard keep-out, optionally inflated by a robot-radius margin.
   */
  public static boolean inKeepout(Translation2d p, double marginMeters) {
    for (Rectangle r : kKeepouts) {
      if (r.expandedBy(marginMeters).contains(p)) {
        return true;
      }
    }
    return false;
  }

  /** True if {@code p} is within the field perimeter. */
  public static boolean onField(Translation2d p) {
    return p.getX() >= 0.0
        && p.getX() <= kFieldLengthMeters
        && p.getY() >= 0.0
        && p.getY() <= kFieldWidthMeters;
  }
}
