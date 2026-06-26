package frc.robot.runtime;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.runtime.reflex.ReflexConstants;
import frc.robot.util.ShotCalculator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.ToDoubleFunction;

/**
 * The named-function registry for the COMPUTED scalar setpoints (shooter RPS, hood rotations).
 * Skills reference these by name in {@code skills.json} — e.g. {@code "shooter":
 * "shooter.shotCalc"} — so the JSON never embeds arithmetic; the math lives here and in {@link
 * ShotCalculator}.
 *
 * <p>{@link ShotCalculator} (interp tables calibrated 2026-03-17, shoot-while-moving behind the NT
 * toggle forced OFF at boot) is reused verbatim. The turret aim is computed by the stateful {@link
 * TurretAiming} reflex, not here, because it carries cross-loop state.
 */
public final class Setpoints {
  private final ShotCalculator shotCalculator = new ShotCalculator();
  private final Map<String, ToDoubleFunction<SetpointContext>> sources = new LinkedHashMap<>();

  /** Everything a computed setpoint can read this loop. */
  public record SetpointContext(
      double distanceToHubMeters, ChassisSpeeds robotRelativeSpeeds, double turretAngleRad) {}

  public Setpoints() {
    // Hub shot: shooter + hood track the interp tables at the shoot-while-moving effective distance
    // (identity when the toggle is off). The distance is the stale-held hub distance from aiming.
    sources.put("shooter.shotCalc", ctx -> shotCalculator.getFlywheelRps(effectiveDistance(ctx)));
    sources.put("hood.shotCalc", ctx -> shotCalculator.getHoodAngleRot(effectiveDistance(ctx)));

    // Passing lob: fixed values (W14).
    sources.put("shooter.passing", ctx -> ReflexConstants.kPassingShooterRps);
    sources.put("hood.passing", ctx -> ReflexConstants.kPassingHoodRot);

    // Unjam: flywheel forward flat-out (W19).
    sources.put("shooter.unjamMax", ctx -> ReflexConstants.kUnjamShooterRps);

    // Off: explicit zero (the shooter clamp floors at 0 anyway).
    sources.put("shooter.off", ctx -> 0.0);
  }

  private double effectiveDistance(SetpointContext ctx) {
    return shotCalculator.effectiveDistance(
        ctx.distanceToHubMeters(), ctx.robotRelativeSpeeds(), ctx.turretAngleRad());
  }

  /** True if a scalar setpoint source with this name is registered. */
  public boolean has(String name) {
    return sources.containsKey(name);
  }

  /** Evaluate a named scalar setpoint source against the current context. */
  public double evaluate(String name, SetpointContext ctx) {
    ToDoubleFunction<SetpointContext> f = sources.get(name);
    if (f == null) {
      throw new IllegalArgumentException("unknown setpoint source '" + name + "'");
    }
    return f.applyAsDouble(ctx);
  }

  public ShotCalculator shotCalculator() {
    return shotCalculator;
  }
}
