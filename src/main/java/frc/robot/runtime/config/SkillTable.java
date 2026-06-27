package frc.robot.runtime.config;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

/**
 * Loads and schema-validates {@code skills.json} with a SAFE FALLBACK (migration constraint). An
 * invalid table reports a clear DriverStation error and {@link #isValid()} is false; the
 * interpreter then forces a hardcoded safe-idle (everything off/hold, intake stowed) rather than
 * running a half-parsed skill table.
 */
public final class SkillTable {
  private static final ObjectMapper kMapper = new ObjectMapper();

  private final Map<String, SkillSpec> byName;
  private final boolean valid;

  private SkillTable(Map<String, SkillSpec> byName, boolean valid) {
    this.byName = byName;
    this.valid = valid;
  }

  public static SkillTable safeFallback() {
    return new SkillTable(new LinkedHashMap<>(), false);
  }

  public static SkillTable load(Path file) {
    try {
      return parse(Files.readString(file));
    } catch (Exception e) {
      DriverStation.reportError(
          "[skills.json] could not read "
              + file
              + " — loading SAFE DEFAULT (safe-idle): "
              + e.getMessage(),
          false);
      return safeFallback();
    }
  }

  public static SkillTable parse(String json) {
    try {
      JsonNode root = kMapper.readTree(json);
      JsonNode list = root.get("skills");
      if (list == null || !list.isArray() || list.isEmpty()) {
        throw new IllegalArgumentException("'skills' must be a non-empty array");
      }
      Map<String, SkillSpec> map = new LinkedHashMap<>();
      for (JsonNode entry : list) {
        SkillSpec spec = parseSkill(entry);
        if (map.containsKey(spec.name())) {
          throw new IllegalArgumentException("duplicate skill '" + spec.name() + "'");
        }
        map.put(spec.name(), spec);
      }
      return new SkillTable(map, true);
    } catch (Exception e) {
      DriverStation.reportError(
          "[skills.json] malformed — loading SAFE DEFAULT (safe-idle): " + e.getMessage(), false);
      return safeFallback();
    }
  }

  private static SkillSpec parseSkill(JsonNode n) {
    String name = req(n, "name").asText();
    String axis = req(n, "axis").asText();
    Set<String> reflexes = new LinkedHashSet<>();
    if (n.has("reflexes")) {
      for (JsonNode r : n.get("reflexes")) {
        reflexes.add(r.asText());
      }
    }
    String done = n.has("done") ? n.get("done").asText() : "never";
    boolean cancelable = !n.has("cancelable") || n.get("cancelable").asBoolean();
    // Optional status-instrumentation timeout; absent = never blocks (POSITIVE_INFINITY).
    double timeoutSeconds =
        n.has("timeoutSeconds") ? n.get("timeoutSeconds").asDouble() : Double.POSITIVE_INFINITY;

    if ("scoring".equals(axis)) {
      // Scoring skills must fully specify their mechanism sources + feed mode.
      return new SkillSpec(
          name,
          axis,
          req(n, "turret").asText(),
          req(n, "hood").asText(),
          req(n, "shooter").asText(),
          req(n, "feed").asText(),
          reflexes,
          done,
          timeoutSeconds,
          cancelable);
    }
    // Non-scoring (intake / drive) axes carry only name + axis; behavior is their reflex.
    return new SkillSpec(name, axis, "", "", "", "", reflexes, done, timeoutSeconds, cancelable);
  }

  private static JsonNode req(JsonNode parent, String field) {
    JsonNode child = parent.get(field);
    if (child == null || child.isNull()) {
      throw new IllegalArgumentException("skill missing required field '" + field + "'");
    }
    return child;
  }

  public boolean isValid() {
    return valid;
  }

  public Optional<SkillSpec> get(String name) {
    return Optional.ofNullable(byName.get(name));
  }

  public int size() {
    return byName.size();
  }
}
