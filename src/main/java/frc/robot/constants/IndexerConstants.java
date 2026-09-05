package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class IndexerConstants {
    private IndexerConstants() {}

    public enum Stage {
        VERTICAL(32, 35.0, InvertedValue.CounterClockwise_Positive),
        HORIZONTAL(31, 35.0, InvertedValue.Clockwise_Positive),
        UPWARD(33, 100.0, InvertedValue.CounterClockwise_Positive);

        public final int canId;
        public final double forwardSpeedRps;
        public final double reverseSpeedRps;
        public final TalonFXConfiguration config;

        Stage(int canId, double forwardSpeedRps, InvertedValue inverted) {
            this.canId = canId;
            this.forwardSpeedRps = forwardSpeedRps;
            this.reverseSpeedRps = -forwardSpeedRps;
            this.config = baseConfig(inverted);
        }
    }

    private static TalonFXConfiguration baseConfig(InvertedValue inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = inverted;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.Slot0.kS = 0.15;
        config.Slot0.kV = 0.12;
        config.Slot0.kP = 0.3;
        config.Voltage.PeakForwardVoltage = 10.0;
        config.Voltage.PeakReverseVoltage = -10.0;
        return config;
    }
}
