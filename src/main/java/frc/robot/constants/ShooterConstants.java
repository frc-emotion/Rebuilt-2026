package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ShooterConstants {
    private ShooterConstants() {}

    public static final int MOTOR_ID = 50;

    public static final double SHOOT_RPS = 55.0;
    public static final double MAX_RPS = 400.0;
    public static final double TOLERANCE_RPS = 1.67;

    public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration();

    static {
        CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        CONFIG.CurrentLimits.StatorCurrentLimit = 160.0;
        CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        CONFIG.CurrentLimits.SupplyCurrentLimit = 120.0;
        CONFIG.Slot0.kS = 0.15;
        CONFIG.Slot0.kV = 0.12;
        CONFIG.Slot0.kP = 0.3;
        CONFIG.Voltage.PeakForwardVoltage = 12.0;
        // Zero reverse voltage is the firmware guarantee that the wheel only ever coasts down.
        CONFIG.Voltage.PeakReverseVoltage = 0.0;
    }
}
