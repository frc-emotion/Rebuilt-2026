package frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

public final class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = 41;
    public static final double CLIMB_GEAR_RATIO = 1.0;

    public static final double TOLERANCE = 0.5;

    public static final double MANUAL_CLIMB_VOLTAGE = 6.0;
    public static final double MANUAL_CLIMB_DEADBAND = 0.1;
    public static final double CLIMB_GRAVITY_COMP_VOLTS = 0.5;

    public static final TalonFXConfiguration CLIMB_CONFIG = new TalonFXConfiguration();

    static {
        // Motor Configuration
        CLIMB_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        CLIMB_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Crucial for climbing!

        // Current Limits - Krakens are powerful, protect the mechanism
        CLIMB_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        CLIMB_CONFIG.CurrentLimits.StatorCurrentLimit = 80.0;
        CLIMB_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        CLIMB_CONFIG.CurrentLimits.SupplyCurrentLimit = 60.0;

        // Feedback
        CLIMB_CONFIG.Feedback.SensorToMechanismRatio = 1.0;

        // Soft Limits (Disabled — manual voltage control, operator stops manually)
        CLIMB_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        CLIMB_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        // Motion Magic PID Slots
        // Slot 0: Position Control
        CLIMB_CONFIG.Slot0.kP = 0.1;
        CLIMB_CONFIG.Slot0.kI = 0.0;
        CLIMB_CONFIG.Slot0.kD = 0.0;
        CLIMB_CONFIG.Slot0.kV = 0.0;
        CLIMB_CONFIG.Slot0.kG = 0.5;
        CLIMB_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        CLIMB_CONFIG.Voltage.PeakForwardVoltage = 10.0;
        CLIMB_CONFIG.Voltage.PeakReverseVoltage = -10.0;

        // Motion Magic constraints
        // CLIMB_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 80.0; // RPS
        // CLIMB_CONFIG.MotionMagic.MotionMagicAcceleration = 160.0; // RPS^2
        // CLIMB_CONFIG.MotionMagic.MotionMagicJerk = 1600.0; // Smoothing

        CLIMB_CONFIG.Feedback.SensorToMechanismRatio = CLIMB_GEAR_RATIO;
        CLIMB_CONFIG.Feedback.RotorToSensorRatio = 1.0;
    }
}
