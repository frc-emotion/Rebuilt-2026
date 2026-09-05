package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class HoodConstants {
    private HoodConstants() {}

    public static final int MOTOR_ID = 52;

    public static final double GEAR_RATIO = 155.0 / 12.0;

    /** Zeroed at boot at the bottom. No hard stop at the top, so MAX is enforced in software and firmware. */
    public static final double MIN_ROT = 0.0;
    public static final double MAX_ROT = 0.08;

    public static final double TOLERANCE_ROT = 0.005;

    public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration();

    static {
        CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        CONFIG.CurrentLimits.StatorCurrentLimit = 40.0;
        CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        CONFIG.CurrentLimits.SupplyCurrentLimit = 20.0;
        CONFIG.Slot0.kP = 100.0;
        CONFIG.Slot0.kI = 50.0;
        CONFIG.Voltage.PeakForwardVoltage = 10.0;
        CONFIG.Voltage.PeakReverseVoltage = -10.0;
        CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        CONFIG.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        CONFIG.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        CONFIG.MotionMagic.MotionMagicAcceleration = 2.0;
        CONFIG.MotionMagic.MotionMagicJerk = 20.0;
        CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ROT;
        CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ROT;
    }
}
