package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.runtime.LocalDriverConstants;
import frc.robot.runtime.Mechanisms;
import frc.robot.runtime.SkillServer;
import frc.robot.runtime.reflex.IndexerFeed;
import frc.robot.runtime.reflex.ReflexConstants;

/**
 * Keeps the robot STANDALONE-functional with no coprocessor: maps the operator controller and the
 * PathPlanner named commands to skill invocations on the {@link SkillServer}, and drives the
 * mechanisms directly in manual mode (where the interpreter commands nothing). The operator
 * bindings are the legacy ones — RT=shoot, LB=pass, right-stick=unjam, A=toggle intake,
 * Start=manual, LT=feed, RB=re-zero turret, plus manual-mode POV/X/Y/B presets — preserved exactly.
 */
public class LocalSkillDriver {
  private final CommandXboxController operator;
  private final SkillServer server;
  private final Mechanisms mechanisms;

  public LocalSkillDriver(
      CommandXboxController operator, SkillServer server, Mechanisms mechanisms) {
    this.operator = operator;
    this.server = server;
    this.mechanisms = mechanisms;
  }

  /** Edge-triggered toggles + the turret re-zero (scheduler-bound). */
  public void configureBindings() {
    // Start: toggle full manual override (persists across disable — held by the SkillServer).
    operator
        .start()
        .onTrue(Commands.runOnce(() -> server.setManualMode(!server.localManualMode())));
    // A: toggle the intake axis (works in every mode).
    operator
        .a()
        .onTrue(Commands.runOnce(() -> server.setIntakeDeploy(!server.localIntakeDeploy())));
    // RB: re-zero the turret at the current physical position (operator mid-match correction).
    operator
        .rightBumper()
        .onTrue(Commands.runOnce(() -> mechanisms.zero("turret")).ignoringDisable(true));
  }

  /**
   * Register PathPlanner named commands (EXACT legacy strings — the .auto files reference them).
   */
  public void registerNamedCommands() {
    NamedCommands.registerCommand(
        "intakeOut", Commands.runOnce(() -> server.setIntakeDeploy(true)));
    NamedCommands.registerCommand(
        "intakeIn", Commands.runOnce(() -> server.setIntakeDeploy(false)));
    // A held SHOOT skill: the feed gate replaces the old pre-spin/ungated-feed split.
    NamedCommands.registerCommand("shoot", heldSkill("shoot"));
    NamedCommands.registerCommand("autoShoot", heldSkill("shoot"));
    NamedCommands.registerCommand("feedIndexers", heldSkill("shoot"));
    NamedCommands.registerCommand("stopAll", Commands.runOnce(server::cancel));
    NamedCommands.registerCommand("reverseIndexer", heldSkill("unjam"));
  }

  private edu.wpi.first.wpilibj2.command.Command heldSkill(String skill) {
    return Commands.startEnd(() -> server.invoke(skill), server::cancel);
  }

  /**
   * Poll the continuous operator inputs once per loop and push the resulting request to the skill
   * server (unless an auto's named command currently holds a skill — autos use {@code invoke}; this
   * is the teleop path). Then, in manual mode, drive the mechanisms directly.
   */
  public void update(boolean teleopActive) {
    if (teleopActive) {
      server.invoke(scoringSkill());
      server.setManualFeed(operator.getLeftTriggerAxis() > 0.5);
    }
    if (server.localManualMode()) {
      applyManualControls();
    }
  }

  private String scoringSkill() {
    if (operator.rightStick().getAsBoolean()) {
      return "unjam";
    }
    boolean shoot = operator.getRightTriggerAxis() > 0.5;
    boolean pass = operator.leftBumper().getAsBoolean();
    if (shoot && pass) {
      return "passShoot";
    }
    if (shoot) {
      return "shoot";
    }
    if (pass) {
      return "passAim";
    }
    return "idle";
  }

  // Manual mechanism control (legacy RobotContainer manual bindings, verbatim priorities). The
  // interpreter commands nothing in manual except the intake axis, so these are the live commands.
  private void applyManualControls() {
    // Turret: POV preset if held, else open-loop jog (firmware soft limits remain the net).
    if (operator.povUp().getAsBoolean()) {
      mechanisms.setPosition("turret", LocalDriverConstants.kTurretPresetForwardRot);
    } else if (operator.povRight().getAsBoolean()) {
      mechanisms.setPosition("turret", LocalDriverConstants.kTurretPresetRightRot);
    } else if (operator.povLeft().getAsBoolean()) {
      mechanisms.setPosition("turret", LocalDriverConstants.kTurretPresetLeftRot);
    } else if (operator.povDown().getAsBoolean()) {
      mechanisms.setPosition("turret", LocalDriverConstants.kTurretPresetBackRot);
    } else {
      double jog =
          MathUtil.applyDeadband(
              operator.getRightX(), LocalDriverConstants.kManualJoystickDeadband);
      mechanisms.setVoltage(
          "turret", MathUtil.clamp(jog, -1, 1) * LocalDriverConstants.kTurretManualVoltsPerUnit);
    }

    // Hood: X/Y/B preset if held, else open-loop jog.
    if (operator.x().getAsBoolean()) {
      mechanisms.setPosition("hood", LocalDriverConstants.kHoodPresetDownRot);
    } else if (operator.y().getAsBoolean()) {
      mechanisms.setPosition("hood", LocalDriverConstants.kHoodPresetMidRot);
    } else if (operator.b().getAsBoolean()) {
      mechanisms.setPosition("hood", LocalDriverConstants.kHoodPresetUpRot);
    } else {
      mechanisms.setVoltage("hood", -operator.getRightY() * LocalDriverConstants.kHoodManualVolts);
    }

    // Shooter + indexer: right-stick unjam wins (it required both subsystems in the legacy
    // bindings, cancelling RT/LT); otherwise RT spins the manual shot and LT runs the vertical
    // feed.
    if (operator.rightStick().getAsBoolean()) {
      mechanisms.setVelocity("shooter", ReflexConstants.kUnjamShooterRps);
      mechanisms.setVelocity(
          IndexerFeed.HORIZONTAL,
          -ReflexConstants.kHorizontalSpeedRps * ReflexConstants.kUnjamFraction);
      mechanisms.setVelocity(
          IndexerFeed.VERTICAL,
          -ReflexConstants.kVerticalSpeedRps * ReflexConstants.kUnjamFraction);
      mechanisms.setVelocity(
          IndexerFeed.UPWARD, -ReflexConstants.kUpwardSpeedRps * ReflexConstants.kUnjamFraction);
    } else {
      if (operator.getRightTriggerAxis() > 0.5) {
        mechanisms.setVelocity("shooter", LocalDriverConstants.kManualShooterRps);
      } else {
        mechanisms.stop("shooter");
      }
      if (operator.getLeftTriggerAxis() > 0.5) {
        mechanisms.setVelocity(IndexerFeed.VERTICAL, ReflexConstants.kVerticalSpeedRps);
      } else {
        mechanisms.stop(IndexerFeed.VERTICAL);
      }
      mechanisms.stop(IndexerFeed.HORIZONTAL);
      mechanisms.stop(IndexerFeed.UPWARD);
    }
  }
}
