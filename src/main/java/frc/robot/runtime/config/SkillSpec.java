package frc.robot.runtime.config;

import java.util.Set;

/**
 * One row of {@code skills.json} (skill-server migration). A scoring-axis skill names its
 * per-mechanism setpoint SOURCES (resolved by {@link frc.robot.runtime.Setpoints} / {@link
 * frc.robot.runtime.TurretAiming}), its indexer FEED mode (resolved by {@link
 * frc.robot.runtime.reflex.IndexerFeed}), the REFLEXES that apply, a DONE predicate, and whether it
 * is cancelable. The intake-axis skill only carries a name + axis (its behavior is the intake
 * reflex). There is no math here — only references into the Java registries.
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
    boolean cancelable) {

  public boolean hasReflex(String reflex) {
    return reflexes.contains(reflex);
  }

  public boolean isScoring() {
    return "scoring".equals(axis);
  }
}
