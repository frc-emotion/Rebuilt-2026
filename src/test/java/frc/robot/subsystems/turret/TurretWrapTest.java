package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.turret.TurretWrap.WrapResult;
import org.junit.jupiter.api.Test;

/** The wrap is the highest-consequence pure logic on the robot (cable chain). Legacy-exact. */
class TurretWrapTest {
  private static final double kRev = TurretConstants.kReverseLimitRot; // -0.73
  private static final double kFwd = TurretConstants.kForwardLimitRot; // 0.39

  @Test
  void wrapAddsOneRotationBelowReverseLimit() {
    // -0.8 is past the reverse limit; +1.0 lands at 0.2, inside range.
    WrapResult result = TurretWrap.apply(-0.8, kRev, kFwd);
    assertEquals(0.2, result.commandedRot(), 1e-12);
    assertTrue(result.wrapped());
  }

  @Test
  void wrapSubtractsOneRotationAboveForwardLimit() {
    // 0.5 is past the forward limit; -1.0 lands at -0.5, inside range.
    WrapResult result = TurretWrap.apply(0.5, kRev, kFwd);
    assertEquals(-0.5, result.commandedRot(), 1e-12);
    assertTrue(result.wrapped());
  }

  @Test
  void wrapResultStillOutOfRangeClamps() {
    // 0.45 wraps to -0.55... in range. Use 1.5: wraps to 0.5, still out of range -> clamps to fwd.
    WrapResult result = TurretWrap.apply(1.5, kRev, kFwd);
    assertEquals(kFwd, result.commandedRot(), 1e-12);
    assertTrue(result.wrapped());
  }

  @Test
  void inRangePassesThrough() {
    WrapResult result = TurretWrap.apply(0.1, kRev, kFwd);
    assertEquals(0.1, result.commandedRot(), 1e-12);
    assertFalse(result.wrapped());
  }

  @Test
  void exactLimitsPassThroughUnwrapped() {
    assertFalse(TurretWrap.apply(kFwd, kRev, kFwd).wrapped());
    assertFalse(TurretWrap.apply(kRev, kRev, kFwd).wrapped());
  }

  @Test
  void valuesMoreThanOneRotationOutClampOnly() {
    // Legacy attempts a SINGLE +/-1 correction (if/else-if, no loop): -2.0 + 1.0 = -1.0 is still
    // out of range and clamps to the reverse limit. Documented legacy behavior, preserved.
    WrapResult result = TurretWrap.apply(-2.0, kRev, kFwd);
    assertEquals(kRev, result.commandedRot(), 1e-12);
    assertTrue(result.wrapped());
  }
}
