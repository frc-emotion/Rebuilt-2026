package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Values copied verbatim from legacy TurretConstants (shooter section). Tuned numbers are sacred.
 */
public final class ShooterConstants {
  private ShooterConstants() {}

  public static final int kMotorId = 50;

  // RPS — prevents flicker at edge of PID settling band
  public static final double kToleranceRps = 1.67;

  public static final double kMaxRps = 400;
  // Fixed manual-mode shot speed (legacy SHOOTER_RPS).
  public static final double kManualRps = 55;

  public static final TalonFXConfiguration kConfig = new TalonFXConfiguration();

  static {
    kConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    kConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kConfig.CurrentLimits.StatorCurrentLimit = 160.0;
    kConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kConfig.CurrentLimits.SupplyCurrentLimit = 120.0;
    kConfig.Slot0.kG = 0.0;
    kConfig.Slot0.kS = 0.15;
    kConfig.Slot0.kV = 0.12;
    kConfig.Slot0.kA = 0.0;
    kConfig.Slot0.kP = 0.3;
    kConfig.Slot0.kI = 0.0;
    kConfig.Slot0.kD = 0.0;

    kConfig.Voltage.PeakForwardVoltage = 12.0;
    kConfig.Voltage.PeakReverseVoltage = 0.0; // never apply reverse voltage — let friction stop it
  }
}
