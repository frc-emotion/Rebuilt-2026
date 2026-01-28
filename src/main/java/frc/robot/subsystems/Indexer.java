package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

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

    public Indexer(){
        horizontalIndexerMotor = new TalonFX(IndexerConstants.horizontalIndexerMotorID);
        verticalIndexerMotor  = new TalonFX(IndexerConstants.verticalIndexerMotorID);
        upwardIndexerMotor = new TalonFX(IndexerConstants.upwardIndexerMotorID);

        configureHorizontalIndexerMotor();
        configureVerticalIndexerMotor();
        configureUpwardIndexerMotor(); 

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
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        horizontalIndexerMotor.getConfigurator().apply(config);
    }

    private void configureVerticalIndexerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        verticalIndexerMotor.getConfigurator().apply(config);
    }

    private void configureUpwardIndexerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        upwardIndexerMotor.getConfigurator().apply(config);



    }

    public void setHorizontalIndexerSpeed(double speed){
        horizontalIndexerMotor.setControl(horizontalIndexerMotorController.withOutput(speed));

    }


    public void setVerticalIndexerSpeed(double speed){
        verticalIndexerMotor.setControl(verticalIndexerMotorController.withOutput(speed));

    }

    public void setUpwardIndexerSpeed(double speed){
        upwardIndexerMotor.setControl(upwardIndexerMotorController.withOutput(speed));

    }

    public void stop(){
        setHorizontalIndexerSpeed(0);
        setVerticalIndexerSpeed(0);
        setUpwardIndexerSpeed(0);
    }
}


