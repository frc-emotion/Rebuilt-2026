package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class IntakeConstants {
    private IntakeConstants() {}

    public static final int PIVOT_MOTOR_ID = 20;
    public static final int ROLLER_MOTOR_ID = 21;
    public static final int PIVOT_ENCODER_ID = 22;

    /** Set so the CANcoder reads 0 at stow: measure raw absolute position at stow and negate it. */
    public static final double PIVOT_ENCODER_MAGNET_OFFSET_ROT = 0.0;
    public static final double PIVOT_ROTOR_TO_SENSOR_RATIO = 27.0;

    public static final double STOWED_POSITION_ROT = 0.0;
    public static final double OUT_POSITION_ROT = 0.3;
    /** Wide on purpose: rollers start slightly before fully out and stop as soon as it leaves the zone. */
    public static final double OUT_TOLERANCE_ROT = 15.0 / 360.0;

    public static final double REVERSE_SOFT_LIMIT_ROT = STOWED_POSITION_ROT - 0.01;
    public static final double FORWARD_SOFT_LIMIT_ROT = OUT_POSITION_ROT + 0.005;

    public static final double ROLLER_SPEED_RPS = 40.0;

    public static final TalonFXConfiguration PIVOT_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration ROLLER_CONFIG = new TalonFXConfiguration();
    public static final CANcoderConfiguration PIVOT_ENCODER_CONFIG = new CANcoderConfiguration();

    static {
        PIVOT_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        PIVOT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        PIVOT_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        PIVOT_CONFIG.CurrentLimits.StatorCurrentLimit = 80.0;
        PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimit = 30.0;
        PIVOT_CONFIG.Slot0.kP = 19.0;
        PIVOT_CONFIG.Slot0.kD = 0.2;
        PIVOT_CONFIG.Voltage.PeakForwardVoltage = 10.0;
        PIVOT_CONFIG.Voltage.PeakReverseVoltage = -10.0;
        PIVOT_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 1.5;
        PIVOT_CONFIG.MotionMagic.MotionMagicAcceleration = 2.5;
        PIVOT_CONFIG.MotionMagic.MotionMagicJerk = 40.0;
        PIVOT_CONFIG.Feedback.FeedbackRemoteSensorID = PIVOT_ENCODER_ID;
        PIVOT_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        PIVOT_CONFIG.Feedback.RotorToSensorRatio = PIVOT_ROTOR_TO_SENSOR_RATIO;
        PIVOT_CONFIG.Feedback.SensorToMechanismRatio = 1.0;
        PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT_ROT;
        PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT_ROT;

        ROLLER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        ROLLER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        ROLLER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        ROLLER_CONFIG.CurrentLimits.StatorCurrentLimit = 60.0;
        ROLLER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        ROLLER_CONFIG.CurrentLimits.SupplyCurrentLimit = 30.0;
        ROLLER_CONFIG.Slot0.kS = 0.15;
        ROLLER_CONFIG.Slot0.kV = 0.12;
        ROLLER_CONFIG.Slot0.kP = 0.3;
        ROLLER_CONFIG.Voltage.PeakForwardVoltage = 10.0;
        ROLLER_CONFIG.Voltage.PeakReverseVoltage = -10.0;

        PIVOT_ENCODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        PIVOT_ENCODER_CONFIG.MagnetSensor.MagnetOffset = PIVOT_ENCODER_MAGNET_OFFSET_ROT;
        PIVOT_ENCODER_CONFIG.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    }
}
