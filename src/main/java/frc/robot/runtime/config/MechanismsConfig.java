package frc.robot.runtime.config;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.runtime.config.MechanismConfig.Sim;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;

/**
 * Loads and schema-validates {@code mechanisms.json} with a SAFE FALLBACK (migration constraint):
 * if the file is missing or malformed, {@link #isValid()} is false, the table is empty, and a clear
 * DriverStation error is reported. The generic mechanism layer treats an invalid config as
 * "construct nothing, command nothing" — mechanisms stay idle and the robot is disabled-safe rather
 * than running a half-parsed table.
 */
public final class MechanismsConfig {
  private static final ObjectMapper kMapper = new ObjectMapper();

  private final Map<String, MechanismConfig> byName;
  private final boolean valid;

  private MechanismsConfig(Map<String, MechanismConfig> byName, boolean valid) {
    this.byName = byName;
    this.valid = valid;
  }

  /** The safe fallback: no mechanisms, marked invalid. */
  public static MechanismsConfig safeFallback() {
    return new MechanismsConfig(new LinkedHashMap<>(), false);
  }

  /**
   * Load from a deploy file. Never throws — a parse failure becomes the safe fallback + an alert.
   */
  public static MechanismsConfig load(Path file) {
    try {
      String json = Files.readString(file);
      return parse(json);
    } catch (Exception e) {
      DriverStation.reportError(
          "[mechanisms.json] could not read "
              + file
              + " — loading SAFE DEFAULT (mechanisms idle): "
              + e.getMessage(),
          false);
      return safeFallback();
    }
  }

  /** Parse + validate from a JSON string (the unit-test entry point). */
  public static MechanismsConfig parse(String json) {
    try {
      JsonNode root = kMapper.readTree(json);
      JsonNode list = require(root, "mechanisms");
      if (!list.isArray() || list.isEmpty()) {
        throw new ConfigException("'mechanisms' must be a non-empty array");
      }
      Map<String, MechanismConfig> map = new LinkedHashMap<>();
      for (JsonNode entry : list) {
        MechanismConfig cfg = parseMechanism(entry);
        if (map.containsKey(cfg.name())) {
          throw new ConfigException("duplicate mechanism name '" + cfg.name() + "'");
        }
        map.put(cfg.name(), cfg);
      }
      return new MechanismsConfig(map, true);
    } catch (Exception e) {
      DriverStation.reportError(
          "[mechanisms.json] malformed — loading SAFE DEFAULT (mechanisms idle): " + e.getMessage(),
          false);
      return safeFallback();
    }
  }

  public boolean isValid() {
    return valid;
  }

  public Optional<MechanismConfig> get(String name) {
    return Optional.ofNullable(byName.get(name));
  }

  public Iterable<MechanismConfig> all() {
    return byName.values();
  }

  public int size() {
    return byName.size();
  }

  // ── Parsing ──────────────────────────────────────────────────────────────

  private static MechanismConfig parseMechanism(JsonNode n) {
    String name = require(n, "name").asText();
    MechanismConfig.ControlType controlType =
        MechanismConfig.ControlType.valueOf(require(n, "controlType").asText().toUpperCase());
    int motorId = require(n, "motorId").asInt();
    String bus = require(n, "bus").asText();
    InvertedValue inverted = invertedOf(require(n, "inverted").asText());
    NeutralModeValue neutral = neutralOf(require(n, "neutralMode").asText());
    double gearRatio = n.has("gearRatio") ? ratio(n.get("gearRatio")) : 1.0;

    JsonNode g = require(n, "gains");
    MechanismConfig.Gains gains =
        new MechanismConfig.Gains(
            dbl(g, "kP"),
            dbl(g, "kI"),
            dbl(g, "kD"),
            dbl(g, "kS"),
            dbl(g, "kV"),
            dbl(g, "kA"),
            dbl(g, "kG"));

    Optional<MechanismConfig.MotionMagic> mm = Optional.empty();
    if (n.has("motionMagic")) {
      JsonNode m = n.get("motionMagic");
      mm =
          Optional.of(
              new MechanismConfig.MotionMagic(
                  dbl(m, "cruiseVelocity"), dbl(m, "acceleration"), dbl(m, "jerk")));
    }

    JsonNode cl = require(n, "currentLimits");
    MechanismConfig.CurrentLimits currentLimits =
        new MechanismConfig.CurrentLimits(
            bool(cl, "statorEnable"),
            dbl(cl, "stator"),
            bool(cl, "supplyEnable"),
            dbl(cl, "supply"));

    double peakFwd = require(n, "peakForwardVoltage").asDouble();
    double peakRev = require(n, "peakReverseVoltage").asDouble();

    Optional<MechanismConfig.SoftLimits> soft = Optional.empty();
    if (n.has("softLimits")) {
      JsonNode s = n.get("softLimits");
      soft =
          Optional.of(
              new MechanismConfig.SoftLimits(
                  bool(s, "forwardEnable"),
                  dbl(s, "forward"),
                  bool(s, "reverseEnable"),
                  dbl(s, "reverse")));
    }

    Optional<MechanismConfig.Feedback> feedback = Optional.empty();
    if (n.has("feedback")) {
      JsonNode f = n.get("feedback");
      feedback =
          Optional.of(
              new MechanismConfig.Feedback(
                  feedbackSourceOf(require(f, "source").asText()),
                  f.has("remoteSensorId") ? f.get("remoteSensorId").asInt() : 0,
                  f.has("rotorToSensorRatio") ? ratio(f.get("rotorToSensorRatio")) : 1.0,
                  f.has("sensorToMechanismRatio") ? ratio(f.get("sensorToMechanismRatio")) : 1.0));
    }

    Optional<MechanismConfig.Encoder> encoder = Optional.empty();
    if (n.has("encoder")) {
      JsonNode e = n.get("encoder");
      encoder =
          Optional.of(
              new MechanismConfig.Encoder(
                  require(e, "id").asInt(),
                  sensorDirectionOf(require(e, "sensorDirection").asText()),
                  e.has("magnetOffset") ? e.get("magnetOffset").asDouble() : 0.0,
                  e.has("fused") && e.get("fused").asBoolean()));
    }

    boolean bootZero = n.has("bootZero") && n.get("bootZero").asBoolean();

    Optional<MechanismConfig.Clamp> clamp = Optional.empty();
    if (n.has("outputClamp")) {
      JsonNode c = n.get("outputClamp");
      clamp = Optional.of(new MechanismConfig.Clamp(dbl(c, "min"), dbl(c, "max")));
    }

    Sim sim = parseSim(require(n, "sim"), name);

    return new MechanismConfig(
        name,
        controlType,
        motorId,
        bus,
        inverted,
        neutral,
        gearRatio,
        gains,
        mm,
        currentLimits,
        peakFwd,
        peakRev,
        soft,
        feedback,
        encoder,
        bootZero,
        clamp,
        sim);
  }

  private static Sim parseSim(JsonNode s, String mechName) {
    Sim.Plant plant = Sim.Plant.valueOf(require(s, "plant").asText().toUpperCase());
    Sim.Controller controller =
        Sim.Controller.valueOf(require(s, "controller").asText().toUpperCase());
    return new Sim(
        plant,
        dbl(s, "moi"),
        s.has("gearing") ? ratio(s.get("gearing")) : 1.0,
        s.has("armLengthMeters") ? s.get("armLengthMeters").asDouble() : 0.0,
        s.has("armMinRot") ? s.get("armMinRot").asDouble() : 0.0,
        s.has("armMaxRot") ? s.get("armMaxRot").asDouble() : 0.0,
        s.has("armStartRot") ? s.get("armStartRot").asDouble() : 0.0,
        s.has("gravity") && s.get("gravity").asBoolean(),
        controller,
        s.has("useKs") && s.get("useKs").asBoolean(),
        s.has("useFeedforward") && s.get("useFeedforward").asBoolean(),
        s.has("pinSoftLimits") && s.get("pinSoftLimits").asBoolean());
  }

  // Gear ratios are stored as exact "a/b" fraction strings (or a number) so they reproduce the
  // legacy Java double bit-for-bit — never pre-divide a tuned ratio into a lossy decimal.
  private static double ratio(JsonNode node) {
    if (node.isNumber()) {
      return node.asDouble();
    }
    String text = node.asText();
    int slash = text.indexOf('/');
    if (slash < 0) {
      return Double.parseDouble(text);
    }
    double num = Double.parseDouble(text.substring(0, slash).trim());
    double den = Double.parseDouble(text.substring(slash + 1).trim());
    return num / den;
  }

  private static InvertedValue invertedOf(String s) {
    return switch (s) {
      case "Clockwise_Positive" -> InvertedValue.Clockwise_Positive;
      case "CounterClockwise_Positive" -> InvertedValue.CounterClockwise_Positive;
      default -> throw new ConfigException("unknown inverted value '" + s + "'");
    };
  }

  private static NeutralModeValue neutralOf(String s) {
    return switch (s) {
      case "Brake" -> NeutralModeValue.Brake;
      case "Coast" -> NeutralModeValue.Coast;
      default -> throw new ConfigException("unknown neutralMode '" + s + "'");
    };
  }

  private static FeedbackSensorSourceValue feedbackSourceOf(String s) {
    return switch (s) {
      case "RotorSensor" -> FeedbackSensorSourceValue.RotorSensor;
      case "RemoteCANcoder" -> FeedbackSensorSourceValue.RemoteCANcoder;
      case "FusedCANcoder" -> FeedbackSensorSourceValue.FusedCANcoder;
      default -> throw new ConfigException("unknown feedback source '" + s + "'");
    };
  }

  private static SensorDirectionValue sensorDirectionOf(String s) {
    return switch (s) {
      case "Clockwise_Positive" -> SensorDirectionValue.Clockwise_Positive;
      case "CounterClockwise_Positive" -> SensorDirectionValue.CounterClockwise_Positive;
      default -> throw new ConfigException("unknown sensorDirection '" + s + "'");
    };
  }

  private static JsonNode require(JsonNode parent, String field) {
    JsonNode child = parent.get(field);
    if (child == null || child.isNull()) {
      throw new ConfigException("missing required field '" + field + "'");
    }
    return child;
  }

  private static double dbl(JsonNode parent, String field) {
    return require(parent, field).asDouble();
  }

  private static boolean bool(JsonNode parent, String field) {
    return require(parent, field).asBoolean();
  }

  /** Thrown internally on a schema violation; always caught and converted to the safe fallback. */
  static final class ConfigException extends RuntimeException {
    ConfigException(String message) {
      super(message);
    }
  }
}
