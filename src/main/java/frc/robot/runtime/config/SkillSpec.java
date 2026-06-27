package frc.robot.runtime.config;

import java.util.Set;

/**
 * One row of {@code skills.json} (skill-server migration). A scoring-axis skill names its
 * per-mechanism setpoint SOURCES (resolved by {@link frc.robot.runtime.Setpoints} / {@link
 * frc.robot.runtime.TurretAiming}), its indexer FEED mode (resolved by {@link
 * frc.robot.runtime.reflex.IndexerFeed}), the REFLEXES that apply, a DONE predicate, an OPTIONAL
 * status {@code timeoutSeconds} (instrumentation only — when the firing skill spends longer than
 * this trying to reach the feeding phase the interpreter reports BLOCKED; it NEVER cancels the
 * skill or alters a mechanism command), and whether it is cancelable. Absent timeout = {@link
 * Double#POSITIVE_INFINITY} (never blocks). The intake-axis skill only carries a name + axis (its
 * behavior is the intake reflex). There is no math here — only references into the Java registries.
 */
public record SkillSpec(
    String name,
    String axis,
    String turret,
    String hood,
    String shooter,
    String feed,
    Set<String> reflexes,
    String done,
    double timeoutSeconds,
    boolean cancelable) {

  public boolean hasReflex(String reflex) {
    return reflexes.contains(reflex);
  }

  public boolean isScoring() {
    return "scoring".equals(axis);
  }
}
