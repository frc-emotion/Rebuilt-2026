package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import static edu.wpi.first.units.Units.*;

@Logged
public class Intake extends SubsystemBase {
    @Logged
    private final TalonFX intakeMotor;
    @Logged
    private final TalonFX rollerMotor;

    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Current> intakeCurrent;
    private final StatusSignal<Voltage> intakeVoltage;


    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Current> rollerCurrent;
    private final StatusSignal<Voltage> rollerVoltage;

    private final MotionMagicVoltage intakeMotionRequest;
    private final VelocityVoltage rollerMotionRequest;

    private Angle currentSetpoint = Degrees.of(0);
    private boolean deployed = false;

    public Intake(CANBus canBus) {
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorID, canBus);
        rollerMotor = new TalonFX(IntakeConstants.rollerMotorID, canBus);

        configureIntakeMotor();
        configureRollerMotor();

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

        intakeMotionRequest = new MotionMagicVoltage(currentSetpoint);
        rollerMotionRequest = new VelocityVoltage(0);


    }

    @Override
    public void periodic() {
        intakeVelocity.refresh();
        intakeVoltage.refresh();
        intakeCurrent.refresh();

        rollerVelocity.refresh();
        rollerVoltage.refresh();
        rollerCurrent.refresh();
    }

    private void configureIntakeMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = intakeMotor.getConfigurator().apply(IntakeConstants.INTAKE_CONFIG, 0.1);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply intake motor configs: " + status.toString());
        }
    }

    private void configureRollerMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = rollerMotor.getConfigurator().apply(IntakeConstants.ROLLER_CONFIG, 0.1);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply roller motor configs: " + status.toString());
        }
    }


    
    public void setIntakeAngle(Angle setpoint) {

        currentSetpoint = setpoint;
        intakeMotor.setControl(intakeMotionRequest.withPosition(setpoint));
    }

    // speed in RPS
    public void setRollerSpeed(double speed) {
        rollerMotor.setControl(rollerMotionRequest.withVelocity(speed));
    }

    public void stopRoller() {
        rollerMotor.setControl(new NeutralOut());
    }

    public boolean atSetpoint(){

        return Math.abs(intakeMotor.getPosition().getValueAsDouble() * 360.0 - currentSetpoint.in(Degrees)) < IntakeConstants.TOLERANCE.in(Degrees);

    }

    // ==================
    // MOTOR ACCESSORS (for FaultMonitor registration)
    // ==================

    public boolean isDeployed() { return deployed; }
    public void setDeployed(boolean deployed) { this.deployed = deployed; }

    public void stop() {
        intakeMotor.stopMotor();
        rollerMotor.stopMotor();
    }

    public TalonFX getIntakeMotor() {
        return intakeMotor;
    }

    public TalonFX getRollerMotor() {
        return rollerMotor;
    }
}
