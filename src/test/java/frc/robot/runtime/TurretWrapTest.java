package frc.robot.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.runtime.TurretWrap.WrapResult;
import org.junit.jupiter.api.Test;

/** The wrap is the highest-consequence pure logic on the robot (cable chain). Legacy-exact (W1). */
class TurretWrapTest {
  // Turret soft limits (mechanisms.json / legacy TurretConstants).
  private static final double kRev = -0.73;
  private static final double kFwd = 0.39;

  @Test
  void wrapAddsOneRotationBelowReverseLimit() {
    WrapResult result = TurretWrap.apply(-0.8, kRev, kFwd);
    assertEquals(0.2, result.commandedRot(), 1e-12);
    assertTrue(result.wrapped());
  }

  @Test
  void wrapSubtractsOneRotationAboveForwardLimit() {
    WrapResult result = TurretWrap.apply(0.5, kRev, kFwd);
    assertEquals(-0.5, result.commandedRot(), 1e-12);
    assertTrue(result.wrapped());
  }

  @Test
  void wrapResultStillOutOfRangeClamps() {
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
    // SINGLE ±1 correction (if/else-if, no loop): -2.0 + 1.0 = -1.0 still out of range -> clamps.
    WrapResult result = TurretWrap.apply(-2.0, kRev, kFwd);
    assertEquals(kRev, result.commandedRot(), 1e-12);
    assertTrue(result.wrapped());
  }
}
