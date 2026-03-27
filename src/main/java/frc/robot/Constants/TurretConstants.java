package frc.robot.Constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class TurretConstants {
    public static final int shooterMotorID = 50;
    public static final int turretMotorID = 51;
    public static final int hoodMotorID = 52;
    public static final int turretEncoderID = 53;
    public static final int hoodEncoderID = 54;

    public static final double TURRET_ENCODER_OFFSET = 0.0;

    // Total gear ratio from motor/CANcoder to turret output (SensorToMechanismRatio).
    // CANcoder is 1:1 with the motor rotor shaft.
    // Empirically measured: 5.08 rotor turns per 1 turret turn.
    public static final double TURRET_GEAR_RATIO = 5.08;
    
    public static final double HOOD_GEAR_RATIO = 155.0 / 12.0; // SensorToMechanismRatio

    // Soft limits relative to boot position (zeroed at startup).
    // Boot position = turret facing straight forward = 0.0.
    // Old absolute: forward=0.08, reverse=-0.50, fwd_limit=0.28
    // New zeroed:   forward=0.00, reverse=-0.58, fwd_limit=0.20
    public static final double TURRET_REVERSE_LIMIT = -0.8;
    public static final double TURRET_FORWARD_LIMIT = 0.05;
    // Turret straight-forward is now 0.0 (zeroed at boot).
    public static final double TURRET_FORWARD_POSITION = 0.0;

    // Offset applied to all turret aim setpoints. If the gear skips again,
    // adjust this single value instead of recalibrating everything.
    public static final double TURRET_AIM_OFFSET = 0.0;

    public static final double HOOD_ENCODER_OFFSET = 0.0;

    // Hood hard stops — zeroed at startup (bottom), max travel = 0.08 rotations
    public static final double HOOD_REVERSE_HARD_STOP = 0.0;
    public static final double HOOD_FORWARD_HARD_STOP = 0.08;

    public static final double shooterTolerance = 2.5; // RPS — prevents flicker at edge of PID settling band
    public static final double turretTolerance = 0.005; // ~1.8° — tight enough for shooting
    public static final double hoodTolerance = 0.005; // ~1.8° — matches turret tolerance

    public static final double MAX_SHOOTER_RPS = 400;
    public static final double SHOOTER_RPS = 55;

    public static TalonFXConfiguration SHOOTER_CONFIG = new TalonFXConfiguration();

    static {
        SHOOTER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        SHOOTER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        SHOOTER_CONFIG.CurrentLimits.StatorCurrentLimit = 120.0;
        SHOOTER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_CONFIG.CurrentLimits.SupplyCurrentLimit = 60.0;
        SHOOTER_CONFIG.Slot0.kG = 0.0;
        SHOOTER_CONFIG.Slot0.kS = 0.15;
        SHOOTER_CONFIG.Slot0.kV = 0.12;
        SHOOTER_CONFIG.Slot0.kA = 0.0;
        SHOOTER_CONFIG.Slot0.kP = 0.3;
        SHOOTER_CONFIG.Slot0.kI = 0.0;
        SHOOTER_CONFIG.Slot0.kD = 0.0;

        SHOOTER_CONFIG.Voltage.PeakForwardVoltage = 10.0;  // cap at 10V to protect motor/battery
        SHOOTER_CONFIG.Voltage.PeakReverseVoltage = 0.0;   // never apply reverse voltage — let friction stop it
    }

    public static TalonFXConfiguration TURRET_CONFIG = new TalonFXConfiguration();

    static {
        TURRET_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        TURRET_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        TURRET_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        TURRET_CONFIG.CurrentLimits.StatorCurrentLimit = 80.0;
        TURRET_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        TURRET_CONFIG.CurrentLimits.SupplyCurrentLimit = 40.0;
        TURRET_CONFIG.Slot0.kG = 0.0;
        TURRET_CONFIG.Slot0.kS = 0.1;
        TURRET_CONFIG.Slot0.kV = 0.0;
        TURRET_CONFIG.Slot0.kA = 0.0;
        TURRET_CONFIG.Slot0.kP = 30;
        TURRET_CONFIG.Slot0.kI = 0;
        TURRET_CONFIG.Slot0.kD = 0.00;

        TURRET_CONFIG.Voltage.PeakForwardVoltage = 10.0;
        TURRET_CONFIG.Voltage.PeakReverseVoltage = -10.0;

        TURRET_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        TURRET_CONFIG.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO; // 5.08 rotor turns per turret turn

        // MotionMagic constraints — prevents turret from slamming. TODO: tune on robot.
        TURRET_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 1.0; // RPS
        TURRET_CONFIG.MotionMagic.MotionMagicAcceleration = 2.0; // RPS^2
        TURRET_CONFIG.MotionMagic.MotionMagicJerk = 20.0; // Smoothing

        TURRET_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        TURRET_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        TURRET_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TURRET_FORWARD_LIMIT;
        TURRET_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TURRET_REVERSE_LIMIT;
    }

    public static TalonFXConfiguration HOOD_CONFIG = new TalonFXConfiguration();

    static {

        HOOD_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        HOOD_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        HOOD_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        HOOD_CONFIG.CurrentLimits.StatorCurrentLimit = 40.0;
        HOOD_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        HOOD_CONFIG.CurrentLimits.SupplyCurrentLimit = 20.0;
        HOOD_CONFIG.Slot0.kG = 0.0;
        HOOD_CONFIG.Slot0.kS = 0.0;
        HOOD_CONFIG.Slot0.kV = 0.0;
        HOOD_CONFIG.Slot0.kA = 0.0;
        HOOD_CONFIG.Slot0.kP = 100;
        HOOD_CONFIG.Slot0.kI = 50;
        HOOD_CONFIG.Slot0.kD = 0.00;

        HOOD_CONFIG.Voltage.PeakForwardVoltage = 10.0;
        HOOD_CONFIG.Voltage.PeakReverseVoltage = -10.0;

        HOOD_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        // MotionMagic constraints — prevents hood from slamming. TODO: tune on robot.
        HOOD_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 1.0; // RPS
        HOOD_CONFIG.MotionMagic.MotionMagicAcceleration = 2.0; // RPS^2
        HOOD_CONFIG.MotionMagic.MotionMagicJerk = 20.0; // Smoothing
        HOOD_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        HOOD_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        HOOD_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HOOD_FORWARD_HARD_STOP;
        HOOD_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HOOD_REVERSE_HARD_STOP;
        HOOD_CONFIG.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;
    }

    public static CANcoderConfiguration TURRET_ENCODER_CONFIG = new CANcoderConfiguration();

    static {

        TURRET_ENCODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        TURRET_ENCODER_CONFIG.MagnetSensor.MagnetOffset = TURRET_ENCODER_OFFSET;
    }

    public static CANcoderConfiguration HOOD_ENCODER_CONFIG = new CANcoderConfiguration();

    static {
        HOOD_ENCODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        HOOD_ENCODER_CONFIG.MagnetSensor.MagnetOffset = HOOD_ENCODER_OFFSET;
    }
}