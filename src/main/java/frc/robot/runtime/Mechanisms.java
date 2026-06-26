package frc.robot.runtime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.runtime.config.MechanismConfig;
import frc.robot.runtime.config.MechanismsConfig;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/**
 * The generic mechanism layer: loads {@code mechanisms.json}, instantiates one {@link
 * MechanismHandle} per non-drive mechanism (real Phoenix 6 or physics sim), and exposes {@code
 * set*}/{@code read} per mechanism by name. This single class replaces the six legacy IO triads
 * (shooter, turret, hood, indexer×3 stages, intake pivot + roller).
 *
 * <p>Output clamps from the config (the legacy per-subsystem clamps, now data — shooter [0,400]
 * RPS, hood [0,0.08] rot) are applied here, and the actually-commanded value is tracked so the
 * predicate registry can answer {@code atSpeed}/{@code commandedNonZero} exactly as the legacy
 * subsystems did.
 *
 * <p>If the config is invalid (missing/malformed file), this layer holds no handles: every {@code
 * set} is a no-op and every {@code read} returns zeros — mechanisms idle, robot disabled-safe.
 */
public class Mechanisms {
  private final Map<String, MechanismHandle> handles = new LinkedHashMap<>();
  private final Map<String, Double> commandedRps = new LinkedHashMap<>();
  private final Map<String, Double> commandedPositionRot = new LinkedHashMap<>();
  private final boolean valid;

  public Mechanisms(MechanismsConfig config, boolean real) {
    this.valid = config.isValid();
    if (!valid) {
      DriverStation.reportError(
          "[Mechanisms] invalid config — running with NO mechanisms (idle, disabled-safe).", false);
      return;
    }
    for (MechanismConfig cfg : config.all()) {
      MechanismHandle handle = real ? new RealMechanism(cfg) : new SimMechanism(cfg);
      handles.put(cfg.name(), handle);
      commandedRps.put(cfg.name(), 0.0);
      commandedPositionRot.put(cfg.name(), 0.0);
    }
  }

  /** Read every mechanism's inputs once per loop (call first, before any decision logic). */
  public void updateInputs() {
    for (MechanismHandle h : handles.values()) {
      h.updateInputs();
    }
  }

  public boolean isValid() {
    return valid;
  }

  public boolean has(String name) {
    return handles.containsKey(name);
  }

  public Set<String> names() {
    return handles.keySet();
  }

  public MechanismState read(String name) {
    MechanismHandle h = handles.get(name);
    return h != null ? h.state() : MechanismState.kZero;
  }

  /** The last velocity (RPS) actually commanded (post-clamp); 0 after a stop. */
  public double commandedRps(String name) {
    return commandedRps.getOrDefault(name, 0.0);
  }

  /** The last position (rotations) actually commanded (post-clamp). */
  public double commandedPositionRot(String name) {
    return commandedPositionRot.getOrDefault(name, 0.0);
  }

  // ── Commands ───────────────────────────────────────────────────────────

  public void setVelocity(String name, double rps) {
    MechanismHandle h = handles.get(name);
    if (h == null) {
      return;
    }
    double commanded = clamp(h.config(), rps);
    h.applyVelocity(commanded);
    commandedRps.put(name, commanded);
  }

  public void setPosition(String name, double rot) {
    setPosition(name, rot, 0.0);
  }

  public void setPosition(String name, double rot, double feedforwardVolts) {
    MechanismHandle h = handles.get(name);
    if (h == null) {
      return;
    }
    double commanded = clamp(h.config(), rot);
    h.applyPosition(commanded, feedforwardVolts);
    commandedPositionRot.put(name, commanded);
  }

  public void setVoltage(String name, double volts) {
    MechanismHandle h = handles.get(name);
    if (h == null) {
      return;
    }
    h.applyVoltage(volts);
    // A manual voltage jog leaves the position setpoint following the current position so leaving
    // manual holds where it is (legacy Hood/Turret semantics).
    commandedPositionRot.put(name, h.state().positionRot());
  }

  public void stop(String name) {
    MechanismHandle h = handles.get(name);
    if (h == null) {
      return;
    }
    h.neutral();
    commandedRps.put(name, 0.0);
  }

  public void stopAll() {
    for (String name : handles.keySet()) {
      stop(name);
    }
  }

  public void zero(String name) {
    MechanismHandle h = handles.get(name);
    if (h == null) {
      return;
    }
    h.zero();
    commandedPositionRot.put(name, 0.0);
  }

  private static double clamp(MechanismConfig config, double value) {
    if (config.outputClamp().isEmpty()) {
      return value;
    }
    MechanismConfig.Clamp c = config.outputClamp().get();
    return MathUtil.clamp(value, c.min(), c.max());
  }
}
