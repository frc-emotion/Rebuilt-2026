package frc.robot.autonomy;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autonomy.FieldGeometry.Rectangle;
import org.junit.jupiter.api.Test;

/** Pure unit tests for the navigation legality sanitizer (no hardware). */
class LegalRegionTest {

  @Test
  void recognizesLegalAndIllegalPoints() {
    assertTrue(LegalRegion.isLegal(new Translation2d(2.6, 4.035)), "open in-zone point is legal");
    assertFalse(LegalRegion.isLegal(FieldGeometry.kBlueHubCenter), "hub center is a keep-out");
    assertFalse(LegalRegion.isLegal(new Translation2d(-1.0, 4.0)), "off-field is illegal");
  }

  @Test
  void sanitizeFreesEveryKeepoutCenter() {
    for (Rectangle r : FieldGeometry.kKeepouts) {
      Translation2d center =
          new Translation2d((r.minX() + r.maxX()) / 2.0, (r.minY() + r.maxY()) / 2.0);
      Translation2d fixed = LegalRegion.sanitize(center, Alliance.Blue);
      assertTrue(
          LegalRegion.isLegal(fixed),
          "sanitize must free keep-out center " + center + " -> " + fixed);
    }
  }

  @Test
  void sanitizeLeavesLegalPointsEssentiallyUnchanged() {
    Translation2d p = new Translation2d(2.6, 4.035);
    Translation2d out = LegalRegion.sanitize(p, Alliance.Blue);
    assertTrue(p.getDistance(out) < 1e-6, "a legal point should pass through unchanged");
  }

  @Test
  void nearestClearDivertsOffAnObstacleOnThePoint() {
    Pose2d desired = new Pose2d(2.6, 4.035, Rotation2d.fromDegrees(0));
    double clearance = 1.05;

    // No obstacle → unchanged.
    assertTrue(
        LegalRegion.nearestClear(desired, java.util.List.of(), clearance).equals(desired),
        "no obstacles → target unchanged");

    // Opponent sitting on the point → divert to a nearby legal, obstacle-free pose.
    Translation2d opponent = new Translation2d(2.6, 4.035);
    Pose2d diverted = LegalRegion.nearestClear(desired, java.util.List.of(opponent), clearance);
    assertFalse(diverted.equals(desired), "must move off the opponent");
    assertTrue(
        LegalRegion.clearOf(diverted.getTranslation(), java.util.List.of(opponent), clearance),
        "diverted point must be clear of the opponent");
    assertTrue(LegalRegion.isLegal(diverted.getTranslation()), "diverted point must be legal");
  }

  @Test
  void sanitizeShootForcesTargetIntoOurAllianceZone() {
    // A shoot pose authored out past the hub (illegal: outside the blue zone) must be pulled back.
    Pose2d wild = new Pose2d(7.0, 4.035, Rotation2d.fromDegrees(10));
    Pose2d fixed = LegalRegion.sanitizeShoot(wild, Alliance.Blue);
    assertTrue(
        FieldGeometry.inAllianceZone(fixed.getTranslation(), Alliance.Blue),
        "shoot target must end up inside our alliance zone, got " + fixed.getTranslation());
    assertTrue(LegalRegion.isLegal(fixed.getTranslation()), "shoot target must be legal");
  }
}
