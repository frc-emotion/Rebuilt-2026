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

    public static final double TURRET_GEAR_RATIO = 122.0/30.0;
    
    public static final double HOOD_GEAR_RATIO = 155.0 / 12.0; // SensorToMechanismRatio
//-0.553955
//0.3884277
    // Dead zone edges (mechanism rotations). Safe range is BETWEEN these values.
    public static final double TURRET_BARRIER_START = 2.65;
    public static final double TURRET_BARRIER_END = 3.4;
    // Safe range: -0.553 to 0.387 (~339° of travel)
    public static final double TURRET_REVERSE_LIMIT = TURRET_BARRIER_START + 0.001;
    public static final double TURRET_FORWARD_LIMIT = TURRET_BARRIER_END - 0.001;
    // Encoder reading (rotations) when turret faces robot-forward.
    // Average of barrier limits (dead zone is behind robot).
    public static final double TURRET_FORWARD_POSITION = (TURRET_BARRIER_START + TURRET_BARRIER_END) / 2.0;

    public static final double HOOD_ENCODER_OFFSET = 0.0;

    // Hood hard stops — absolute positions the hood must NEVER exceed
    // hood low: 0.087158 (raw rotations)
    // hood high: 0.011962 (raw rotations)
    public static final double HOOD_REVERSE_HARD_STOP = 0.011962; // TODO: set to actual min
    public static final double HOOD_FORWARD_HARD_STOP = 0.087158; // TODO: set to actual max

    public static final double shooterTolerance = 0.5;
    public static final double turretTolerance = 0.05;
    public static final double hoodTolerance = 0.001;

    public static final double MAX_SHOOTER_RPS = 100;

    public static TalonFXConfiguration SHOOTER_CONFIG = new TalonFXConfiguration();

    static {
        SHOOTER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        SHOOTER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        SHOOTER_CONFIG.Slot0.kG = 0.0;
        SHOOTER_CONFIG.Slot0.kS = 0.0;
        SHOOTER_CONFIG.Slot0.kV = 0.0;
        SHOOTER_CONFIG.Slot0.kA = 0.0;
        SHOOTER_CONFIG.Slot0.kP = 0.1;
        SHOOTER_CONFIG.Slot0.kI = 0.0;
        SHOOTER_CONFIG.Slot0.kD = 0.00;
    }

    public static TalonFXConfiguration TURRET_CONFIG = new TalonFXConfiguration();

    static {
        TURRET_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        TURRET_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        TURRET_CONFIG.Slot0.kG = 0.0;
        TURRET_CONFIG.Slot0.kS = 0.5;
        TURRET_CONFIG.Slot0.kV = 0.12;
        TURRET_CONFIG.Slot0.kA = 0.0;
        TURRET_CONFIG.Slot0.kP = 0.01;
        TURRET_CONFIG.Slot0.kI = 0.0;
        TURRET_CONFIG.Slot0.kD = 0.5;

        TURRET_CONFIG.Feedback.FeedbackRemoteSensorID = turretEncoderID;
        TURRET_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        TURRET_CONFIG.Feedback.SensorToMechanismRatio = 1.0; // CANcoder is on mechanism output
        TURRET_CONFIG.Feedback.RotorToSensorRatio = TURRET_GEAR_RATIO; // 122:30 motor-to-CANcoder

        // MotionMagic constraints — prevents turret from slamming. TODO: tune on robot.
        TURRET_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 2.0; // RPS
        TURRET_CONFIG.MotionMagic.MotionMagicAcceleration = 4.0; // RPS^2
        TURRET_CONFIG.MotionMagic.MotionMagicJerk = 40.0; // Smoothing

        TURRET_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        TURRET_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        TURRET_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TURRET_FORWARD_LIMIT;
        TURRET_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TURRET_REVERSE_LIMIT;
    }

    public static TalonFXConfiguration HOOD_CONFIG = new TalonFXConfiguration();

    static {
        HOOD_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        HOOD_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        HOOD_CONFIG.Slot0.kG = 0.0;
        HOOD_CONFIG.Slot0.kS = 0.5;
        HOOD_CONFIG.Slot0.kV = 0.0;
        HOOD_CONFIG.Slot0.kA = 0.0;
        HOOD_CONFIG.Slot0.kP = 0.01;
        HOOD_CONFIG.Slot0.kI = 0.0;
        HOOD_CONFIG.Slot0.kD = 0.00;

        HOOD_CONFIG.Feedback.FeedbackRemoteSensorID = hoodEncoderID;
        HOOD_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        HOOD_CONFIG.Feedback.RotorToSensorRatio = 1.0;

        // MotionMagic constraints — prevents hood from slamming. TODO: tune on robot.
        HOOD_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 1.0; // RPS
        HOOD_CONFIG.MotionMagic.MotionMagicAcceleration = 2.0; // RPS^2
        HOOD_CONFIG.MotionMagic.MotionMagicJerk = 20.0; // Smoothing
        HOOD_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        HOOD_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        HOOD_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HOOD_FORWARD_HARD_STOP;
        HOOD_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HOOD_REVERSE_HARD_STOP;
        HOOD_CONFIG.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;
        HOOD_CONFIG.Feedback.RotorToSensorRatio = 15.0 / 12.0;
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