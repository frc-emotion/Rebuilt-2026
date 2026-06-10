package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/**
 * Values copied verbatim from legacy TurretConstants (turret section). Tuned numbers are sacred.
 */
public final class TurretConstants {
  private TurretConstants() {}

  public static final int kMotorId = 51;
  public static final int kEncoderId = 53;

  public static final double kEncoderOffset = 0.0;

  // Total gear ratio from motor to turret output (SensorToMechanismRatio).
  // Empirically measured: 5.08 rotor turns per 1 turret turn; 122/24 is the tooth-count guess.
  public static final double kGearRatio = 122.0 / 24.0; // ??/18 //5.08

  // Soft limits relative to boot position (zeroed at startup, turret facing straight forward).
  public static final double kReverseLimitRot = -0.73;
  public static final double kForwardLimitRot = 0.39;

  // Offset applied to all turret aim setpoints. If the gear skips again,
  // adjust this single value instead of recalibrating everything.
  public static final double kAimOffsetRot = 0.0;

  public static final double kToleranceRot = 0.005; // ~1.8° — tight enough for shooting

  // Manual jog: joystick input × this many volts (legacy setTurretVoltage scale).
  public static final double kManualVoltsPerUnit = 3.0;

  // Chassis-omega lead compensation multiplier — sampled+logged but compensation disabled in
  // legacy (W4); kept for future retuning.
  public static final double kOmegaFeedforwardMultiplier = 0.05;

  // D-pad setpoints (rotations). The legacy "+18°" comment on RIGHT was wrong; 0.25 rot = +90°
  // and the VALUE is team-confirmed correct.
  public static final double kPresetForwardRot = 0.0;
  public static final double kPresetRightRot = 0.25;
  public static final double kPresetLeftRot = -0.250;
  public static final double kPresetBackRot = -0.500;

  public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();

  static {
    kConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    kConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    kConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    kConfig.Slot0.kG = 0.0;
    kConfig.Slot0.kS = 0.2; // 0.1
    kConfig.Slot0.kV = 0.0;
    kConfig.Slot0.kA = 0.0;
    // kConfig.Slot0.kP = 30;
    kConfig.Slot0.kP = 40;
    kConfig.Slot0.kI = 0;
    kConfig.Slot0.kD = 0.00;

    kConfig.Voltage.PeakForwardVoltage = 10.0;
    kConfig.Voltage.PeakReverseVoltage = -10.0;

    kConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    kConfig.Feedback.SensorToMechanismRatio = kGearRatio;

    // MotionMagic constraints — prevents turret from slamming. TODO: tune on robot.
    kConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0; // RPS
    kConfig.MotionMagic.MotionMagicAcceleration = 2.0; // RPS^2
    kConfig.MotionMagic.MotionMagicJerk = 20.0; // Smoothing

    kConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    kConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    kConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kForwardLimitRot;
    kConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kReverseLimitRot;
  }

  public static final CANcoderConfiguration kEncoderConfig = new CANcoderConfiguration();

  static {
    kEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    kEncoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;
  }
}
