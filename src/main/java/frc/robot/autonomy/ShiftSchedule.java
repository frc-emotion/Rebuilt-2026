package frc.robot.autonomy;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/**
 * The 2026 "REBUILT" teleop SHIFT clock as pure logic. Teleop (2:20 → 0:00, 140 s) is split into a
 * 10 s TRANSITION SHIFT, four 25 s ALLIANCE SHIFTS, and a 30 s END GAME. During the ALLIANCE SHIFTS
 * only ONE alliance's HUB is active at a time (FUEL in an inactive HUB scores nothing); the order
 * is set by which alliance scored more FUEL in AUTO and then alternates. TRANSITION and END GAME
 * have both HUBS active.
 *
 * <p>Everything here is a pure function of (teleop time remaining) and (the FMS game-specific
 * message + our alliance) so it is hardware-free and unit-tested. The strategy layer turns the
 * resulting {@link Mode} into behavior; this class never touches a mechanism or the navigator.
 *
 * <p>SAFE FALLBACK: the exact encoding of the AUTO-winner game data is published on the FRC Control
 * System website (not the manual), so {@link #parseOurHubInactiveFirst} is an isolated, documented
 * best-guess. When it cannot parse the message it returns empty, and {@link #modeOf} then treats
 * our HUB as ACTIVE for every shift — i.e. the robot degrades to a plain score-and-cycle that is
 * always legal, never harvesting through a shift it should have scored.
 */
public final class ShiftSchedule {
  private ShiftSchedule() {}

  /** Teleop segment. {@code PRE_MATCH} = not in teleop yet (auto/disabled). */
  public enum Shift {
    PRE_MATCH,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    ENDGAME
  }

  /** What the strategy should be doing right now. */
  public enum Mode {
    OUR_HUB_ACTIVE,
    OUR_HUB_INACTIVE,
    ENDGAME
  }

  // ── Durations (manual Table 6-2). Teleop = 10 + 4×25 + 30 = 140 s (displayed 2:20 → 0:00). ──
  public static final double kTeleopLengthSeconds = 140.0;
  public static final double kTransitionSeconds = 10.0;
  public static final double kAllianceShiftSeconds = 25.0;
  public static final double kEndgameSeconds = 30.0;

  // Boundaries expressed as teleop time REMAINING (DriverStation.getMatchTime() counts down the
  // current period). Transition is the first 10 s, so it ends at 130 s remaining; endgame is the
  // last 30 s.
  private static final double kTransitionEndRemaining = kTeleopLengthSeconds - kTransitionSeconds;
  private static final double kShift1EndRemaining = kTransitionEndRemaining - kAllianceShiftSeconds;
  private static final double kShift2EndRemaining = kShift1EndRemaining - kAllianceShiftSeconds;
  private static final double kShift3EndRemaining = kShift2EndRemaining - kAllianceShiftSeconds;
  private static final double kShift4EndRemaining = kShift3EndRemaining - kAllianceShiftSeconds;

  /**
   * Map teleop time remaining (seconds, as {@code DriverStation.getMatchTime()} reports it during
   * teleop) to the current {@link Shift}. A non-positive / unknown remaining time reads as {@code
   * PRE_MATCH}.
   */
  public static Shift shiftOf(double teleopRemainingSeconds) {
    double t = teleopRemainingSeconds;
    if (t <= 0.0 || t > kTeleopLengthSeconds) {
      return Shift.PRE_MATCH;
    }
    if (t > kTransitionEndRemaining) {
      return Shift.TRANSITION;
    }
    if (t > kShift1EndRemaining) {
      return Shift.SHIFT_1;
    }
    if (t > kShift2EndRemaining) {
      return Shift.SHIFT_2;
    }
    if (t > kShift3EndRemaining) {
      return Shift.SHIFT_3;
    }
    if (t > kShift4EndRemaining) {
      return Shift.SHIFT_4;
    }
    return Shift.ENDGAME;
  }

  /**
   * Is OUR hub active during {@code shift}? TRANSITION / END GAME / PRE_MATCH are always active.
   * The four alliance shifts alternate, seeded by whether our hub is the one inactive in SHIFT 1.
   */
  public static boolean ourHubActive(Shift shift, boolean ourHubInactiveInShift1) {
    return switch (shift) {
      case PRE_MATCH, TRANSITION, ENDGAME -> true;
      case SHIFT_1 -> !ourHubInactiveInShift1;
      case SHIFT_2 -> ourHubInactiveInShift1;
      case SHIFT_3 -> !ourHubInactiveInShift1;
      case SHIFT_4 -> ourHubInactiveInShift1;
    };
  }

  /**
   * The strategy mode for a teleop time + known shift order. When {@code ourHubInactiveInShift1} is
   * empty (game data not yet parsed), our hub is treated as ACTIVE every shift — the safe,
   * always-legal score-and-cycle fallback.
   */
  public static Mode modeOf(
      double teleopRemainingSeconds, Optional<Boolean> ourHubInactiveInShift1) {
    Shift shift = shiftOf(teleopRemainingSeconds);
    if (shift == Shift.ENDGAME) {
      return Mode.ENDGAME;
    }
    boolean active = ourHubActive(shift, ourHubInactiveInShift1.orElse(false));
    return active ? Mode.OUR_HUB_ACTIVE : Mode.OUR_HUB_INACTIVE;
  }

  /**
   * Parse the dashboard {@code AutonomyModeOverride} string into a forced {@link Mode} for
   * sim/debug (so you can watch each behavior without a practice match). {@code
   * MATCH}/blank/unknown → empty (use the real shift clock); {@code ACTIVE} / {@code INACTIVE} /
   * {@code ENDGAME} force that mode.
   */
  public static Optional<Mode> parseModeOverride(String value) {
    if (value == null) {
      return Optional.empty();
    }
    return switch (value.trim().toUpperCase(java.util.Locale.ROOT)) {
      case "ACTIVE", "OUR_HUB_ACTIVE" -> Optional.of(Mode.OUR_HUB_ACTIVE);
      case "INACTIVE", "OUR_HUB_INACTIVE" -> Optional.of(Mode.OUR_HUB_INACTIVE);
      case "ENDGAME" -> Optional.of(Mode.ENDGAME);
      default -> Optional.empty();
    };
  }

  /**
   * Seconds until our hub NEXT becomes active (0 if it is active now, or the order is unknown). An
   * inactive ALLIANCE SHIFT is always followed by an active period, so this is just the time left
   * in the current (inactive) shift. The harvest plan uses this to know when to stop collecting and
   * stage home.
   */
  public static double secondsUntilActive(
      double teleopRemainingSeconds, Optional<Boolean> ourHubInactiveInShift1) {
    Shift shift = shiftOf(teleopRemainingSeconds);
    if (shift == Shift.ENDGAME || ourHubActive(shift, ourHubInactiveInShift1.orElse(false))) {
      return 0.0;
    }
    return Math.max(0.0, teleopRemainingSeconds - shiftEndRemaining(shift));
  }

  /** Teleop time remaining at the END of {@code shift} (when it hands off to the next). */
  private static double shiftEndRemaining(Shift shift) {
    return switch (shift) {
      case TRANSITION -> kTransitionEndRemaining;
      case SHIFT_1 -> kShift1EndRemaining;
      case SHIFT_2 -> kShift2EndRemaining;
      case SHIFT_3 -> kShift3EndRemaining;
      case SHIFT_4 -> kShift4EndRemaining;
      case PRE_MATCH, ENDGAME -> 0.0;
    };
  }

  /**
   * Best-guess parse of the FMS game-specific message into "is our hub the one inactive in SHIFT
   * 1?". The manual states FMS relays the AUTO FUEL winner (whose hub is inactive in SHIFT 1) to
   * the operator console at teleop start, but the exact wire format lives on the Control System
   * site, so this is a single, isolated assumption: the first character names the INACTIVE-first
   * alliance — {@code 'R'} red, {@code 'B'} blue (case-insensitive). Anything else → empty (caller
   * falls back to always-active). Update only this method once the real format is known.
   */
  public static Optional<Boolean> parseOurHubInactiveFirst(String gameMessage, Alliance ours) {
    if (gameMessage == null || gameMessage.isEmpty()) {
      return Optional.empty();
    }
    char c = Character.toUpperCase(gameMessage.charAt(0));
    Alliance inactiveFirst;
    if (c == 'R') {
      inactiveFirst = Alliance.Red;
    } else if (c == 'B') {
      inactiveFirst = Alliance.Blue;
    } else {
      return Optional.empty();
    }
    return Optional.of(inactiveFirst == ours);
  }
}
