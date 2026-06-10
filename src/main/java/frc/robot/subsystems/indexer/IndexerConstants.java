package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Values copied verbatim from legacy IndexerConstants. Tuned numbers are sacred. */
public final class IndexerConstants {
  private IndexerConstants() {}

  public static final int kHorizontalMotorId = 31;
  public static final int kVerticalMotorId = 32;
  public static final int kUpwardMotorId = 33;

  public static final double kHorizontalSpeedRps = 35;
  public static final double kVerticalSpeedRps = 35;
  public static final double kUpwardSpeedRps = 100;

  // Vertical feed while intaking at rest = 75% of full vertical speed (legacy indexerDefault).
  public static final double kIntakingVerticalSpeedRps = kVerticalSpeedRps * 0.75;

  // Unjam runs every stage backwards at half speed (legacy reverseIndexers).
  public static final double kUnjamFraction = 0.5;

  // CLEARING (shot release) reverses every stage at FULL speed for this long (team decision).
  public static final double kClearingSeconds = 2.0;

  public static final TalonFXConfiguration kHorizontalConfig = new TalonFXConfiguration();

  static {
    kHorizontalConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kHorizontalConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kHorizontalConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kHorizontalConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    kHorizontalConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kHorizontalConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    kHorizontalConfig.Slot0.kS = 0.15;
    kHorizontalConfig.Slot0.kV = 0.12;
    kHorizontalConfig.Slot0.kP = 0.3;

    kHorizontalConfig.Voltage.PeakForwardVoltage = 10.0;
    kHorizontalConfig.Voltage.PeakReverseVoltage = -10.0;
  }

  public static final TalonFXConfiguration kVerticalConfig = new TalonFXConfiguration();

  static {
    kVerticalConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kVerticalConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kVerticalConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kVerticalConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    kVerticalConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kVerticalConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    kVerticalConfig.Slot0.kS = 0.15;
    kVerticalConfig.Slot0.kV = 0.12;
    kVerticalConfig.Slot0.kP = 0.3;

    kVerticalConfig.Voltage.PeakForwardVoltage = 10.0;
    kVerticalConfig.Voltage.PeakReverseVoltage = -10.0;
  }

  public static final TalonFXConfiguration kUpwardConfig = new TalonFXConfiguration();

  static {
    kUpwardConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kUpwardConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kUpwardConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kUpwardConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    kUpwardConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kUpwardConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    kUpwardConfig.Slot0.kS = 0.15;
    kUpwardConfig.Slot0.kV = 0.12;
    kUpwardConfig.Slot0.kP = 0.3;

    kUpwardConfig.Voltage.PeakForwardVoltage = 10.0;
    kUpwardConfig.Voltage.PeakReverseVoltage = -10.0;
  }
}
