package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;

import static edu.wpi.first.units.Units.Seconds;
import frc.robot.Constants.IntakeConstants;
//diameter ball wheel 2 inch diamter 
//diamter conveyer bell 1.125

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//gear reduction 1:1 wheels 
public class Intake extends SubsystemBase {
    //Config for Hopper
    private final TalonFX intakeMotor;
    private final TalonFX rollerMotor;


    private final DutyCycleOut intakeController = new DutyCycleOut(0);
    private final DutyCycleOut rollerController = new DutyCycleOut(0);

    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Current> intakeCurrent; 
    private final StatusSignal<Voltage> intakeVoltage;

    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Current> rollerCurrent;
    private final StatusSignal<Voltage> rollerVoltage;

    public Intake(){
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorID);
        rollerMotor = new TalonFX(IntakeConstants.rollerMotorID);

        configureIntakeMotor();
        configureRollerMotor();

        //Intialize status signals
        intakeVelocity = intakeMotor.getVelocity();
        intakeCurrent = intakeMotor.getSupplyCurrent();
        intakeVoltage = intakeMotor.getMotorVoltage();


        rollerVelocity = rollerMotor.getVelocity();
        rollerCurrent = rollerMotor.getSupplyCurrent();
        rollerVoltage = rollerMotor.getMotorVoltage();

        intakeVelocity.setUpdateFrequency(50); // 50 Hz
        intakeVoltage.setUpdateFrequency(50);
        intakeCurrent.setUpdateFrequency(50);
        
        rollerVelocity.setUpdateFrequency(50);
        rollerVoltage.setUpdateFrequency(50);
        rollerCurrent.setUpdateFrequency(50);


    }

    public void periodic(){
        intakeVelocity.refresh();
        intakeVoltage.refresh();
        intakeCurrent.refresh();
        
        rollerVelocity.refresh();
        rollerVoltage.refresh();
        rollerCurrent.refresh();
    }
    
    private void configureIntakeMotor(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        intakeMotor.getConfigurator().apply(config);

    }

    private void configureRollerMotor(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        rollerMotor.getConfigurator().apply(config);
    }

    public void setIntakeSpeed(double speed){
        // speed needs to be given in [-1,1]
        intakeMotor.setControl(intakeController.withOutput(speed));
    }

    public void setRollerSpeed(double speed){
        //speed needs to be given in [-1,1]
        rollerMotor.setControl(rollerController.withOutput(speed));
    }

    public boolean getCurrentSpike(){

        return (intakeMotor.getSupplyCurrent().getValueAsDouble() > IntakeConstants.IntakeCurrentSpike); 
    }

    // public void runIntake(double intakeSpeed, double rollerSpeed){
    //     setIntakeSpeed(intakeSpeed);
    //     setRollerSpeed(rollerSpeed);
    // }

    public void stopIntake(){
        setIntakeSpeed(0);
    }

    public void stopRoller(){
        setRollerSpeed(0);
    }
}
