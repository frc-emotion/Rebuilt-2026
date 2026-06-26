package frc.robot.runtime;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LocalSkillDriver;
import frc.robot.runtime.SkillInterpreter.InterpreterInputs;
import frc.robot.runtime.perception.PerceptionProvider;
import frc.robot.runtime.reflex.SafetyReflex;

/**
 * The single robot-loop driver for the skill-server runtime: each loop it reads every mechanism's
 * inputs once, polls the local operator request, resolves the active request (local or
 * coprocessor), runs the stateless interpreter, and publishes status + the state estimate. It is a
 * WPILib subsystem so its {@code periodic()} runs on the 20 ms loop; perception (the Vision
 * subsystem) is constructed BEFORE this one so its periodic runs first (registration order) and the
 * interpreter reads a fresh frame.
 */
@Logged
public class RuntimeSubsystem extends SubsystemBase {
  private final Mechanisms mechanisms;
  private final SkillInterpreter interpreter;
  private final SkillServer server;
  private final LocalSkillDriver driver;
  private final PerceptionProvider perception;

  public RuntimeSubsystem(
      Mechanisms mechanisms,
      SkillInterpreter interpreter,
      SkillServer server,
      LocalSkillDriver driver,
      PerceptionProvider perception) {
    this.mechanisms = mechanisms;
    this.interpreter = interpreter;
    this.server = server;
    this.driver = driver;
    this.perception = perception;
  }

  /** Re-seed at enable (teleopInit/autonomousInit). Manual mode persists across disable. */
  public void onEnable() {
    interpreter.onEnable();
  }

  public SkillInterpreter interpreter() {
    return interpreter;
  }

  public SkillServer server() {
    return server;
  }

  public Mechanisms mechanisms() {
    return mechanisms;
  }

  @Override
  public void periodic() {
    mechanisms.updateInputs();
    driver.update(DriverStation.isTeleopEnabled());
    InterpreterInputs inputs = server.resolve();
    interpreter.periodic(inputs);
    server.publish(interpreter, perception);

    // Disabled-output lockout: hold every mechanism neutral while disabled (the interpreter's
    // per-skill commands above are not actuated). Real-robot safety net; inert when enabled.
    if (SafetyReflex.isDisabled()) {
      SafetyReflex.lockout(mechanisms);
    }
  }
}
