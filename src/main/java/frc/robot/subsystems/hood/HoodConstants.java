package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/** Values copied verbatim from legacy TurretConstants (hood section). Tuned numbers are sacred. */
public final class HoodConstants {
  private HoodConstants() {}

  public static final int kMotorId = 52;
  public static final int kEncoderId = 54;

  public static final double kGearRatio = 155.0 / 12.0; // SensorToMechanismRatio
  public static final double kEncoderOffset = 0.0;

  // Hood hard stops — zeroed at startup (bottom), max travel = 0.08 rotations
  public static final double kReverseHardStopRot = 0.0;
  public static final double kForwardHardStopRot = 0.08;

  public static final double kToleranceRot = 0.005; // ~1.8° — matches turret tolerance

  // Operator presets (X/Y/B): range [0.0, 0.08]
  public static final double kPresetDownRot = 0.005;
  public static final double kPresetMidRot = 0.040;
  public static final double kPresetUpRot = 0.070;

  // Fixed lob for the passing shot (legacy hardcoded values, W14).
  public static final double kPassingAngleRot = 0.067;

  // Manual-mode jog scale (new in manual mode; firmware soft limits remain the safety net).
  public static final double kManualVolts = 2.0;

  public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();

  static {
    kConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    kConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    kConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    kConfig.Slot0.kG = 0.0;
    kConfig.Slot0.kS = 0.0;
    kConfig.Slot0.kV = 0.0;
    kConfig.Slot0.kA = 0.0;
    kConfig.Slot0.kP = 100;
    kConfig.Slot0.kI = 50;
    kConfig.Slot0.kD = 0.00;

    kConfig.Voltage.PeakForwardVoltage = 10.0;
    kConfig.Voltage.PeakReverseVoltage = -10.0;

    kConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    kConfig.Feedback.SensorToMechanismRatio = kGearRatio;

    // MotionMagic constraints — prevents hood from slamming. TODO: tune on robot.
    kConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0; // RPS
    kConfig.MotionMagic.MotionMagicAcceleration = 2.0; // RPS^2
    kConfig.MotionMagic.MotionMagicJerk = 20.0; // Smoothing

    kConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    kConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    kConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kForwardHardStopRot;
    kConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kReverseHardStopRot;
  }

  public static final CANcoderConfiguration kEncoderConfig = new CANcoderConfiguration();

  static {
    kEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    kEncoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;
  }
}
