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

    public static final double HORIZONTAL_INDEXER_SPEED = 50;

    static{
        HORIZONTAL_INDEXER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        HORIZONTAL_INDEXER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        HORIZONTAL_INDEXER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        HORIZONTAL_INDEXER_CONFIG.CurrentLimits.StatorCurrentLimit = 60.0;
        HORIZONTAL_INDEXER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        HORIZONTAL_INDEXER_CONFIG.CurrentLimits.SupplyCurrentLimit = 30.0;
        HORIZONTAL_INDEXER_CONFIG.Slot0.kS = 0.15;
        HORIZONTAL_INDEXER_CONFIG.Slot0.kV = 0.12;
        HORIZONTAL_INDEXER_CONFIG.Slot0.kP = 0.3;
        
    }



    public static final TalonFXConfiguration VERTICAL_INDEXER_CONFIG = new TalonFXConfiguration();

    public static final double VERTICAL_INDEXER_SPEED = 50;

    static{
        VERTICAL_INDEXER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        VERTICAL_INDEXER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        VERTICAL_INDEXER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        VERTICAL_INDEXER_CONFIG.CurrentLimits.StatorCurrentLimit = 60.0;
        VERTICAL_INDEXER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        VERTICAL_INDEXER_CONFIG.CurrentLimits.SupplyCurrentLimit = 30.0;
        VERTICAL_INDEXER_CONFIG.Slot0.kS = 0.15;
        VERTICAL_INDEXER_CONFIG.Slot0.kV = 0.12;
        VERTICAL_INDEXER_CONFIG.Slot0.kP = 0.3;
        
    }

    public static final TalonFXConfiguration UPWARD_INDEXER_CONFIG = new TalonFXConfiguration();

    public static final double UPWARD_INDEXER_SPEED = 50;

    static{
        UPWARD_INDEXER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        UPWARD_INDEXER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        UPWARD_INDEXER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        UPWARD_INDEXER_CONFIG.CurrentLimits.StatorCurrentLimit = 60.0;
        UPWARD_INDEXER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        UPWARD_INDEXER_CONFIG.CurrentLimits.SupplyCurrentLimit = 30.0;
        UPWARD_INDEXER_CONFIG.Slot0.kS = 0.15;
        UPWARD_INDEXER_CONFIG.Slot0.kV = 0.12;
        UPWARD_INDEXER_CONFIG.Slot0.kP = 0.3;
        
    }



}
