package frc.robot.runtime;

/**
 * Tuned numbers for the LOCAL operator driver (manual jog scales, presets, the manual shot speed),
 * ported VERBATIM from the legacy TurretConstants / HoodConstants / ShooterConstants /
 * Superstructure. These drive the operator seam only; the coprocessor seam (SkillServer) does not
 * use them.
 */
public final class LocalDriverConstants {
  private LocalDriverConstants() {}

  public static final double kManualJoystickDeadband = 0.08; // legacy Superstructure
  public static final double kTurretManualVoltsPerUnit = 3.0; // legacy TurretConstants
  public static final double kHoodManualVolts = 2.0; // legacy HoodConstants
  public static final double kManualShooterRps = 55; // legacy ShooterConstants.kManualRps

  // Turret POV presets (rotations). VALUES are law: 0.25 rot = +90° (the legacy "+18°" comment
  // lied).
  public static final double kTurretPresetForwardRot = 0.0;
  public static final double kTurretPresetRightRot = 0.25;
  public static final double kTurretPresetLeftRot = -0.250;
  public static final double kTurretPresetBackRot = -0.500;

  // Hood X/Y/B presets (rotations).
  public static final double kHoodPresetDownRot = 0.005;
  public static final double kHoodPresetMidRot = 0.040;
  public static final double kHoodPresetUpRot = 0.070;
}
