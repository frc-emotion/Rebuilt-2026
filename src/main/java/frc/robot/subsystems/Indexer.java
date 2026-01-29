package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;

public class Indexer extends SubsystemBase {
    private final TalonFX horizontalIndexerMotor;
    private final TalonFX verticalIndexerMotor; 
    private final TalonFX upwardIndexerMotor; 


    private final DutyCycleOut horizontalIndexerMotorController = new DutyCycleOut(0);
    private final DutyCycleOut verticalIndexerMotorController  = new DutyCycleOut(0);
    private final DutyCycleOut upwardIndexerMotorController = new DutyCycleOut(0);

    private final StatusSignal<AngularVelocity> horizontalIndexerMotorVelocity;
    private final StatusSignal<Current> horizontalIndexerMotorCurrent; 
    private final StatusSignal<Voltage> horizontalIndexerMotorVoltage;
    
    private final StatusSignal<AngularVelocity> verticalIndexerMotorVelocity;
    private final StatusSignal<Current> verticalIndexerMotorCurrent; 
    private final StatusSignal<Voltage> verticalIndexerMotorVoltage;

    private final StatusSignal<AngularVelocity> upwardIndexerMotorVelocity;
    private final StatusSignal<Current> upwardIndexerMotorCurrent; 
    private final StatusSignal<Voltage> upwardIndexerMotorVoltage;

    private final MotionMagicVelocityVoltage horizontalMotionController;
    private final MotionMagicVelocityVoltage verticalMotionController;
    private final MotionMagicVelocityVoltage upwardMotionController;

    public Indexer(){
        horizontalIndexerMotor = new TalonFX(IndexerConstants.horizontalIndexerMotorID);
        verticalIndexerMotor  = new TalonFX(IndexerConstants.verticalIndexerMotorID);
        upwardIndexerMotor = new TalonFX(IndexerConstants.upwardIndexerMotorID);

        configureHorizontalIndexerMotor();
        configureVerticalIndexerMotor();
        configureUpwardIndexerMotor(); 

        horizontalMotionController = new MotionMagicVelocityVoltage(0);
        verticalMotionController = new MotionMagicVelocityVoltage(0);
        upwardMotionController = new MotionMagicVelocityVoltage(0);

        horizontalIndexerMotorVelocity = horizontalIndexerMotor.getVelocity();
        horizontalIndexerMotorCurrent = horizontalIndexerMotor.getSupplyCurrent();
        horizontalIndexerMotorVoltage = horizontalIndexerMotor.getMotorVoltage();

        verticalIndexerMotorVelocity = verticalIndexerMotor.getVelocity();
        verticalIndexerMotorCurrent = verticalIndexerMotor.getSupplyCurrent();
        verticalIndexerMotorVoltage = verticalIndexerMotor.getMotorVoltage();

        upwardIndexerMotorVelocity = upwardIndexerMotor.getVelocity();
        upwardIndexerMotorCurrent = upwardIndexerMotor.getSupplyCurrent();
        upwardIndexerMotorVoltage = upwardIndexerMotor.getMotorVoltage();


        horizontalIndexerMotorVelocity.setUpdateFrequency(50);
        horizontalIndexerMotorCurrent.setUpdateFrequency(50);
        horizontalIndexerMotorVoltage.setUpdateFrequency(50);

        verticalIndexerMotorVelocity.setUpdateFrequency(50);
        verticalIndexerMotorCurrent.setUpdateFrequency(50);
        verticalIndexerMotorVoltage.setUpdateFrequency(50);

        upwardIndexerMotorVelocity.setUpdateFrequency(50);
        upwardIndexerMotorCurrent.setUpdateFrequency(50);
        upwardIndexerMotorVoltage.setUpdateFrequency(50);


    }

    

    public void periodic(){
        horizontalIndexerMotorVelocity.refresh();
        horizontalIndexerMotorCurrent.refresh();
        horizontalIndexerMotorVoltage.refresh();

        verticalIndexerMotorVelocity.refresh();
        verticalIndexerMotorCurrent.refresh();
        verticalIndexerMotorVoltage.refresh();

        upwardIndexerMotorVelocity.refresh();
        upwardIndexerMotorCurrent.refresh();
        upwardIndexerMotorVoltage.refresh();
    }


    private void configureHorizontalIndexerMotor() {
        horizontalIndexerMotor.getConfigurator().apply(IndexerConstants.HORIZONTAL_INDEXER_CONFIG);
    }

    private void configureVerticalIndexerMotor() {
        verticalIndexerMotor.getConfigurator().apply(IndexerConstants.VERTICAL_INDEXER_CONFIG);
    }

    private void configureUpwardIndexerMotor() {
        upwardIndexerMotor.getConfigurator().apply(IndexerConstants.UPWARD_INDEXER_CONFIG);
    }


    public void setIndexerSpeed(double speed, IndexerType indexer){
        switch(indexer){
            case VERTICAL:
                verticalMotionController.withVelocity(speed);
            case HORIZONTAL:
                horizontalMotionController.withVelocity(speed);
            case UPWARD:
                upwardMotionController.withVelocity(speed);
        }
    }
}


