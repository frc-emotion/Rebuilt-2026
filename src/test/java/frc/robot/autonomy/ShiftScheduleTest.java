package frc.robot.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autonomy.ShiftSchedule.Mode;
import frc.robot.autonomy.ShiftSchedule.Shift;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/** Pure unit tests for the teleop SHIFT clock + hub-active derivation (no hardware). */
class ShiftScheduleTest {

  @Test
  void shiftBoundariesFromTeleopRemaining() {
    assertEquals(Shift.PRE_MATCH, ShiftSchedule.shiftOf(-1.0));
    assertEquals(Shift.PRE_MATCH, ShiftSchedule.shiftOf(0.0));
    assertEquals(Shift.PRE_MATCH, ShiftSchedule.shiftOf(141.0));
    assertEquals(Shift.TRANSITION, ShiftSchedule.shiftOf(140.0));
    assertEquals(Shift.TRANSITION, ShiftSchedule.shiftOf(131.0));
    assertEquals(Shift.SHIFT_1, ShiftSchedule.shiftOf(130.0)); // transition ends at 130
    assertEquals(Shift.SHIFT_1, ShiftSchedule.shiftOf(106.0));
    assertEquals(Shift.SHIFT_2, ShiftSchedule.shiftOf(105.0));
    assertEquals(Shift.SHIFT_3, ShiftSchedule.shiftOf(80.0));
    assertEquals(Shift.SHIFT_4, ShiftSchedule.shiftOf(55.0));
    assertEquals(Shift.SHIFT_4, ShiftSchedule.shiftOf(31.0));
    assertEquals(Shift.ENDGAME, ShiftSchedule.shiftOf(30.0)); // endgame is the last 30 s
    assertEquals(Shift.ENDGAME, ShiftSchedule.shiftOf(1.0));
  }

  @Test
  void hubActiveAlternatesAfterShift1() {
    // Our hub inactive in SHIFT 1 → active 2, inactive 3, active 4.
    assertTrue(ShiftSchedule.ourHubActive(Shift.TRANSITION, true));
    assertFalse(ShiftSchedule.ourHubActive(Shift.SHIFT_1, true));
    assertTrue(ShiftSchedule.ourHubActive(Shift.SHIFT_2, true));
    assertFalse(ShiftSchedule.ourHubActive(Shift.SHIFT_3, true));
    assertTrue(ShiftSchedule.ourHubActive(Shift.SHIFT_4, true));
    assertTrue(ShiftSchedule.ourHubActive(Shift.ENDGAME, true));
    // Mirror: our hub active in SHIFT 1.
    assertTrue(ShiftSchedule.ourHubActive(Shift.SHIFT_1, false));
    assertFalse(ShiftSchedule.ourHubActive(Shift.SHIFT_2, false));
  }

  @Test
  void modeFoldsShiftAndHubState() {
    assertEquals(Mode.OUR_HUB_ACTIVE, ShiftSchedule.modeOf(135.0, Optional.of(true))); // transition
    assertEquals(Mode.OUR_HUB_INACTIVE, ShiftSchedule.modeOf(120.0, Optional.of(true))); // shift 1
    assertEquals(Mode.OUR_HUB_ACTIVE, ShiftSchedule.modeOf(120.0, Optional.of(false)));
    assertEquals(Mode.ENDGAME, ShiftSchedule.modeOf(20.0, Optional.of(true)));
    // Unknown game data → always-active safe fallback.
    assertEquals(Mode.OUR_HUB_ACTIVE, ShiftSchedule.modeOf(120.0, Optional.empty()));
  }

  @Test
  void secondsUntilActiveCountsDownTheInactiveShift() {
    assertEquals(
        15.0, ShiftSchedule.secondsUntilActive(120.0, Optional.of(true)), 1e-9); // shift1 ->105
    assertEquals(5.0, ShiftSchedule.secondsUntilActive(110.0, Optional.of(true)), 1e-9);
    assertEquals(
        0.0, ShiftSchedule.secondsUntilActive(120.0, Optional.of(false)), 1e-9); // active now
    assertEquals(
        0.0, ShiftSchedule.secondsUntilActive(135.0, Optional.empty()), 1e-9); // transition
    assertEquals(0.0, ShiftSchedule.secondsUntilActive(20.0, Optional.of(true)), 1e-9); // endgame
  }

  @Test
  void parseGameDataIsAllianceRelativeWithSafeFallback() {
    assertEquals(Optional.of(true), ShiftSchedule.parseOurHubInactiveFirst("R", Alliance.Red));
    assertEquals(Optional.of(false), ShiftSchedule.parseOurHubInactiveFirst("R", Alliance.Blue));
    assertEquals(Optional.of(true), ShiftSchedule.parseOurHubInactiveFirst("b", Alliance.Blue));
    assertEquals(Optional.empty(), ShiftSchedule.parseOurHubInactiveFirst("", Alliance.Blue));
    assertEquals(Optional.empty(), ShiftSchedule.parseOurHubInactiveFirst(null, Alliance.Blue));
    assertEquals(Optional.empty(), ShiftSchedule.parseOurHubInactiveFirst("X", Alliance.Blue));
  }

  @Test
  void parseModeOverrideMapsDashboardString() {
    assertEquals(Optional.empty(), ShiftSchedule.parseModeOverride("MATCH"));
    assertEquals(Optional.empty(), ShiftSchedule.parseModeOverride(""));
    assertEquals(Optional.empty(), ShiftSchedule.parseModeOverride(null));
    assertEquals(Optional.of(Mode.OUR_HUB_ACTIVE), ShiftSchedule.parseModeOverride("active"));
    assertEquals(Optional.of(Mode.OUR_HUB_INACTIVE), ShiftSchedule.parseModeOverride(" INACTIVE "));
    assertEquals(Optional.of(Mode.ENDGAME), ShiftSchedule.parseModeOverride("Endgame"));
  }
}
