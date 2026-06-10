package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void tablesClampAtEndpoints() {
    ShotCalculator calc = new ShotCalculator();
    // Below the 1.55 m first row and above the 4.62 m last row, lookups clamp (legacy behavior:
    // shots between 4.62 and 7.0 m are "valid" but use the 4.62 m values).
    assertEquals(41.5, calc.getFlywheelRps(0.5), 1e-9);
    assertEquals(56.5, calc.getFlywheelRps(6.5), 1e-9);
    assertEquals(0.000, calc.getHoodAngleRot(0.5), 1e-9);
    assertEquals(0.035, calc.getHoodAngleRot(6.5), 1e-9);
  }

  @Test
  void tablesInterpolateBetweenCalibratedPoints() {
    ShotCalculator calc = new ShotCalculator();
    // Exact calibrated points return exact values.
    assertEquals(43.5, calc.getFlywheelRps(2.47698), 1e-9);
    assertEquals(0.03, calc.getHoodAngleRot(3.5), 1e-9);
    // A midpoint lands strictly between its neighbors.
    double mid = calc.getFlywheelRps(3.0);
    assertTrue(mid > 43.5 && mid < 48.5, "interpolated value out of bracket: " + mid);
  }

  @Test
  void validityWindowIsOneToSevenMeters() {
    ShotCalculator calc = new ShotCalculator();
    assertFalse(calc.isValidDistance(0.9));
    assertTrue(calc.isValidDistance(1.0));
    assertTrue(calc.isValidDistance(7.0));
    assertFalse(calc.isValidDistance(7.1));
  }

  @Test
  void effectiveDistanceIdentityWhenToggleOff() {
    ShotCalculator calc = new ShotCalculator(); // ctor force-sets the toggle OFF
    ChassisSpeeds closingFast = new ChassisSpeeds(3.0, 0.0, 0.0);
    assertEquals(4.0, calc.effectiveDistance(4.0, closingFast, 0.0), 1e-9);
  }

  @Test
  void effectiveDistanceShortensWhenClosingWithToggleOn() {
    ShotCalculator calc = new ShotCalculator();
    NetworkTableInstance.getDefault()
        .getTable("Tuning")
        .getBooleanTopic("ShootWhileMovingEnabled")
        .getEntry(false)
        .set(true);
    // 2 m/s straight at the target (turret at 0 rad): 4.0 − 2.0×0.5 = 3.0.
    ChassisSpeeds closing = new ChassisSpeeds(2.0, 0.0, 0.0);
    assertEquals(3.0, calc.effectiveDistance(4.0, closing, 0.0), 1e-9);
    // Moving away lengthens.
    ChassisSpeeds opening = new ChassisSpeeds(-2.0, 0.0, 0.0);
    assertEquals(5.0, calc.effectiveDistance(4.0, opening, 0.0), 1e-9);
    // Restore the default for any test that follows.
    NetworkTableInstance.getDefault()
        .getTable("Tuning")
        .getBooleanTopic("ShootWhileMovingEnabled")
        .getEntry(false)
        .set(false);
  }
}
