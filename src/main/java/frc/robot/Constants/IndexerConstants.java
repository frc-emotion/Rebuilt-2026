package frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class IndexerConstants {

    public enum IndexerType {
        VERTICAL,
        HORIZONTAL,
        UPWARD
    }

    public static final int horizontalIndexerMotorID = 31;
    public static final int verticalIndexerMotorID = 32;
    public static final int upwardIndexerMotorID = 33;


    public static final TalonFXConfiguration HORIZONTAL_INDEXER_CONFIG = new TalonFXConfiguration();

    public static final double HORIZONTAL_INDEXER_SPEED = 30;

    static{
        HORIZONTAL_INDEXER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        HORIZONTAL_INDEXER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        HORIZONTAL_INDEXER_CONFIG.Slot0.kG = 0; // Volts to overcome gravity
        HORIZONTAL_INDEXER_CONFIG.Slot0.kS = 0; // Volts to overcome static friction
        HORIZONTAL_INDEXER_CONFIG.Slot0.kV = 0.0; // Volts for a velocity target of 1 rps
        HORIZONTAL_INDEXER_CONFIG.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/s
        HORIZONTAL_INDEXER_CONFIG.Slot0.kP = 0.1;
        HORIZONTAL_INDEXER_CONFIG.Slot0.kI = 0.0;
        HORIZONTAL_INDEXER_CONFIG.Slot0.kD = 0.00;
        
    }



    public static final TalonFXConfiguration VERTICAL_INDEXER_CONFIG = new TalonFXConfiguration();

    public static final double VERTICAL_INDEXER_SPEED = 30;

    static{
        VERTICAL_INDEXER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        VERTICAL_INDEXER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        VERTICAL_INDEXER_CONFIG.Slot0.kG = 0; // Volts to overcome gravity
        VERTICAL_INDEXER_CONFIG.Slot0.kS = 0; // Volts to overcome static friction
        VERTICAL_INDEXER_CONFIG.Slot0.kV = 0.0; // Volts for a velocity target of 1 rps
        VERTICAL_INDEXER_CONFIG.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/s
        VERTICAL_INDEXER_CONFIG.Slot0.kP = 0.1;
        VERTICAL_INDEXER_CONFIG.Slot0.kI = 0.0;
        VERTICAL_INDEXER_CONFIG.Slot0.kD = 0.00;
        
    }

    public static final TalonFXConfiguration UPWARD_INDEXER_CONFIG = new TalonFXConfiguration();

    public static final double UPWARD_INDEXER_SPEED = 100;

    static{
        UPWARD_INDEXER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        UPWARD_INDEXER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        UPWARD_INDEXER_CONFIG.Slot0.kG = 0; // Volts to overcome gravity
        UPWARD_INDEXER_CONFIG.Slot0.kS = 0; // Volts to overcome static friction
        UPWARD_INDEXER_CONFIG.Slot0.kV = 0.0; // Volts for a velocity target of 1 rps
        UPWARD_INDEXER_CONFIG.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/s
        UPWARD_INDEXER_CONFIG.Slot0.kP = 0.1;
        UPWARD_INDEXER_CONFIG.Slot0.kI = 0.0;
        UPWARD_INDEXER_CONFIG.Slot0.kD = 0.00;
        
    }



}
