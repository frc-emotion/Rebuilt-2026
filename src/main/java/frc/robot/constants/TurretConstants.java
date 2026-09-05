package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class TurretConstants {
    private TurretConstants() {}

    public static final int MOTOR_ID = 51;

    public static final double GEAR_RATIO = 122.0 / 24.0;

    /** Limits relative to the boot position (turret facing forward = 0). Span exceeds one rotation so wrapping always finds a reachable setpoint. */
    public static final double FORWARD_LIMIT_ROT = 0.39;
    public static final double REVERSE_LIMIT_ROT = -0.73;

    public static final double TOLERANCE_ROT = 0.005;

    public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration();

    static {
        CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        CONFIG.CurrentLimits.StatorCurrentLimit = 80.0;
        CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        CONFIG.CurrentLimits.SupplyCurrentLimit = 40.0;
        CONFIG.Slot0.kS = 0.2;
        CONFIG.Slot0.kP = 40.0;
        CONFIG.Voltage.PeakForwardVoltage = 10.0;
        CONFIG.Voltage.PeakReverseVoltage = -10.0;
        CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        CONFIG.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        CONFIG.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        CONFIG.MotionMagic.MotionMagicAcceleration = 2.0;
        CONFIG.MotionMagic.MotionMagicJerk = 20.0;
        CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_LIMIT_ROT;
        CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_LIMIT_ROT;
    }
}
