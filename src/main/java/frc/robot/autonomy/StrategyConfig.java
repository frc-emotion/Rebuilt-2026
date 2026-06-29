package frc.robot.autonomy;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.autonomy.ShiftSchedule.Mode;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * Loads and schema-validates {@code strategy.json} — the editable, per-match teleop strategy — with
 * the same SAFE FALLBACK contract as {@code skills.json} / {@code mechanisms.json}. An invalid file
 * reports a DriverStation error and {@link #isValid()} is false; the strategy brain then degrades
 * to a hardcoded legal score-and-cycle rather than running a half-parsed plan.
 *
 * <p>The file names WAYPOINTS ({@link Location}) and assigns each match {@link Mode} a {@link Plan}
 * (a behavior + a route of waypoint names). This loader only validates structure and resolves
 * names; it embeds no field knowledge and no control flow — {@link MatchTree} interprets the plans
 * and {@link LegalRegion} clamps every resolved pose into legal space.
 */
public final class StrategyConfig {
  private static final ObjectMapper kMapper = new ObjectMapper();
  private static final double kDefaultReturnLeadSeconds = 4.5;

  /** A named blue-origin waypoint. */
  public record Location(double x, double y, double headingDeg) {
    public Pose2d toPose() {
      return new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg));
    }
  }

  /**
   * One mode's behavior. {@code shootFrom} + {@code collectRoute} drive the score-and-cycle modes;
   * {@code harvestRoute} + {@code stageAt} drive the off-shift harvest. Unused fields are empty for
   * a given mode.
   */
  public record Plan(
      Optional<String> shootFrom,
      List<String> collectRoute,
      List<String> harvestRoute,
      Optional<String> stageAt) {}

  private final double returnLeadSeconds;
  private final Map<String, Location> locations;
  private final Map<Mode, Plan> plans;
  private final boolean valid;

  private StrategyConfig(
      double returnLeadSeconds,
      Map<String, Location> locations,
      Map<Mode, Plan> plans,
      boolean valid) {
    this.returnLeadSeconds = returnLeadSeconds;
    this.locations = locations;
    this.plans = plans;
    this.valid = valid;
  }

  public static StrategyConfig safeFallback() {
    return new StrategyConfig(
        kDefaultReturnLeadSeconds, new LinkedHashMap<>(), new EnumMap<>(Mode.class), false);
  }

  public static StrategyConfig load(Path file) {
    try {
      return parse(Files.readString(file));
    } catch (Exception e) {
      DriverStation.reportError(
          "[strategy.json] could not read "
              + file
              + " — loading SAFE DEFAULT (score-and-cycle): "
              + e.getMessage(),
          false);
      return safeFallback();
    }
  }

  public static StrategyConfig parse(String json) {
    try {
      JsonNode root = kMapper.readTree(json);

      double lead =
          root.has("returnLeadSeconds")
              ? root.get("returnLeadSeconds").asDouble()
              : kDefaultReturnLeadSeconds;

      JsonNode locsNode = root.get("locations");
      if (locsNode == null || !locsNode.isObject() || locsNode.isEmpty()) {
        throw new IllegalArgumentException("'locations' must be a non-empty object");
      }
      Map<String, Location> locs = new LinkedHashMap<>();
      for (Map.Entry<String, JsonNode> e : locsNode.properties()) {
        JsonNode v = e.getValue();
        locs.put(
            e.getKey(),
            new Location(
                req(v, "x").asDouble(),
                req(v, "y").asDouble(),
                v.has("headingDeg") ? v.get("headingDeg").asDouble() : 0.0));
      }

      JsonNode plansNode = root.get("plans");
      if (plansNode == null || !plansNode.isObject() || plansNode.isEmpty()) {
        throw new IllegalArgumentException("'plans' must be a non-empty object");
      }
      Map<Mode, Plan> plans = new EnumMap<>(Mode.class);
      for (Map.Entry<String, JsonNode> e : plansNode.properties()) {
        Mode mode = Mode.valueOf(e.getKey());
        plans.put(mode, parsePlan(e.getValue(), locs));
      }

      return new StrategyConfig(lead, locs, plans, true);
    } catch (Exception e) {
      DriverStation.reportError(
          "[strategy.json] malformed — loading SAFE DEFAULT (score-and-cycle): " + e.getMessage(),
          false);
      return safeFallback();
    }
  }

  private static Plan parsePlan(JsonNode n, Map<String, Location> locs) {
    Optional<String> shootFrom = optName(n, "shootFrom", locs);
    Optional<String> stageAt = optName(n, "stageAt", locs);
    List<String> collect = route(n, "collectRoute", locs);
    List<String> harvest = route(n, "harvestRoute", locs);
    return new Plan(shootFrom, collect, harvest, stageAt);
  }

  private static Optional<String> optName(JsonNode n, String field, Map<String, Location> locs) {
    if (!n.has(field) || n.get(field).isNull()) {
      return Optional.empty();
    }
    String name = n.get(field).asText();
    if (!locs.containsKey(name)) {
      throw new IllegalArgumentException(
          "'" + field + "' references unknown location '" + name + "'");
    }
    return Optional.of(name);
  }

  private static List<String> route(JsonNode n, String field, Map<String, Location> locs) {
    List<String> out = new ArrayList<>();
    if (n.has(field)) {
      for (JsonNode e : n.get(field)) {
        String name = e.asText();
        if (!locs.containsKey(name)) {
          throw new IllegalArgumentException(
              "'" + field + "' references unknown location '" + name + "'");
        }
        out.add(name);
      }
    }
    return out;
  }

  private static JsonNode req(JsonNode parent, String field) {
    JsonNode child = parent.get(field);
    if (child == null || child.isNull()) {
      throw new IllegalArgumentException("location missing required field '" + field + "'");
    }
    return child;
  }

  // ── Accessors ──

  public boolean isValid() {
    return valid;
  }

  public double returnLeadSeconds() {
    return returnLeadSeconds;
  }

  public Optional<Pose2d> pose(String name) {
    Location l = locations.get(name);
    return l == null ? Optional.empty() : Optional.of(l.toPose());
  }

  public Optional<Plan> plan(Mode mode) {
    return Optional.ofNullable(plans.get(mode));
  }

  /** Resolve a list of waypoint names to poses, skipping any that no longer exist. */
  public List<Pose2d> route(List<String> names) {
    List<Pose2d> out = new ArrayList<>();
    for (String name : names) {
      pose(name).ifPresent(out::add);
    }
    return out;
  }
}
