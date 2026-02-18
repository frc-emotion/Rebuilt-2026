package frc.robot.Constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class TurretConstants {
    public static final int shooterMotorID = 1001;
    public static final int turretMotorID = 1002;
    public static final int hoodMotorID = 1003;
    public static final int turretEncoderID = 1004;
    public static final int hoodEncoderID = 1005; 

    public static final double TURRET_ENCODER_OFFSET = 0.0; 
    public static final double TURRET_GEAR_RATIO = 1.0;

    public static final double HOOD_ENCODER_OFFSET = 0.0;
    public static final double HOOD_GEAR_RATIO = 1.0;

    public static final double shooterTolerance = 0.5; 
    public static final double turretTolerance = 0.05; 
    public static final double hoodTolerance = 0.02;

    public static final double MAX_SHOOTER_RPS = 80.0; 

    public static TalonFXConfiguration SHOOTER_CONFIG = new TalonFXConfiguration();
    
    static{
        SHOOTER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        SHOOTER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        SHOOTER_CONFIG.Slot0.kG = 0.53;
        SHOOTER_CONFIG.Slot0.kS = 0.5;
        SHOOTER_CONFIG.Slot0.kV = 0.0;
        SHOOTER_CONFIG.Slot0.kA = 0.0;
        SHOOTER_CONFIG.Slot0.kP = 25;
        SHOOTER_CONFIG.Slot0.kI = 0.0;
        SHOOTER_CONFIG.Slot0.kD = 0.00;
    }

    public static TalonFXConfiguration TURRET_CONFIG = new TalonFXConfiguration();

    static{
        TURRET_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        TURRET_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        TURRET_CONFIG.Slot0.kG = 0.0; 
        TURRET_CONFIG.Slot0.kS = 0.1; 
        TURRET_CONFIG.Slot0.kV = 0.12; 
        TURRET_CONFIG.Slot0.kA = 0.0;
        TURRET_CONFIG.Slot0.kP = 50; 
        TURRET_CONFIG.Slot0.kI = 0.0;
        TURRET_CONFIG.Slot0.kD = 0.5; 
        
        
        TURRET_CONFIG.Feedback.FeedbackRemoteSensorID = turretEncoderID;
        TURRET_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        TURRET_CONFIG.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO;
        TURRET_CONFIG.Feedback.RotorToSensorRatio = 1.0; 
    }

    public static TalonFXConfiguration HOOD_CONFIG = new TalonFXConfiguration();

    static {
        HOOD_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        HOOD_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        HOOD_CONFIG.Slot0.kG = 0.53;
        HOOD_CONFIG.Slot0.kS = 0.5;
        HOOD_CONFIG.Slot0.kV = 0.0;
        HOOD_CONFIG.Slot0.kA = 0.0;
        HOOD_CONFIG.Slot0.kP = 25;
        HOOD_CONFIG.Slot0.kI = 0.0;
        HOOD_CONFIG.Slot0.kD = 0.00;


        HOOD_CONFIG.Feedback.FeedbackRemoteSensorID = hoodEncoderID;
        HOOD_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        HOOD_CONFIG.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO;
        HOOD_CONFIG.Feedback.RotorToSensorRatio = 1.0; 
    }

    public static CANcoderConfiguration TURRET_ENCODER_CONFIG = new CANcoderConfiguration(); 

    static {

        TURRET_ENCODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        TURRET_ENCODER_CONFIG.MagnetSensor.MagnetOffset = TURRET_ENCODER_OFFSET; 
    }

    public static CANcoderConfiguration HOOD_ENCODER_CONFIG = new CANcoderConfiguration();

    static{
        HOOD_ENCODER_CONFIG.MagnetSensor.SensorDirection  = SensorDirectionValue.Clockwise_Positive; 
        HOOD_ENCODER_CONFIG.MagnetSensor.MagnetOffset = HOOD_ENCODER_OFFSET;
    }
}