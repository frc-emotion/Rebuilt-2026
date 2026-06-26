package frc.robot.runtime;

/**
 * The narrow command+read surface a reflex needs from the mechanism layer. {@link Mechanisms}
 * implements it; extracting it lets the stateful reflexes (notably the intake nested machine) be
 * unit-tested against a fake with a directly settable position, exactly as the legacy IO triads
 * were.
 */
public interface MechanismCommander {

  MechanismState read(String name);

  void setVelocity(String name, double rps);

  void setPosition(String name, double rot);

  void stop(String name);
}
