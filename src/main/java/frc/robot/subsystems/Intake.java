package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import static edu.wpi.first.units.Units.*;

/**
 * Intake subsystem with automatic telemetry via Epilogue.
 * 
 * <p>
 * The @Logged annotation on TalonFX motors enables automatic logging of
 * motor health data (current, voltage, temperature) via TalonFXLogger.
 */
@Logged
public class Intake extends SubsystemBase {
    // Motors marked @Logged will have health data auto-logged via TalonFXLogger
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
    private final MotionMagicVelocityVoltage rollerMotionRequest;

    private Angle currentSetpoint = Degrees.of(0);

    public Intake(CANBus canBus) {
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorID, canBus);
        rollerMotor = new TalonFX(IntakeConstants.rollerMotorID, canBus);

        configureIntakeMotor();
        configureRollerMotor();

        // Intialize status signals
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
        rollerMotionRequest = new MotionMagicVelocityVoltage(0);


    }

    public void periodic() {
        intakeVelocity.refresh();
        intakeVoltage.refresh();
        intakeCurrent.refresh();

        rollerVelocity.refresh();
        rollerVoltage.refresh();
        rollerCurrent.refresh();
    }

    private void configureIntakeMotor() {

        intakeMotor.getConfigurator().apply(IntakeConstants.INTAKE_CONFIG);

    }

    private void configureRollerMotor() {

        rollerMotor.getConfigurator().apply(IntakeConstants.ROLLER_CONFIG);
    }


    
    public void setIntakeAngle(Angle setpoint) {

        // passing in angle as degrees
        currentSetpoint = setpoint;
        intakeMotor.setControl(intakeMotionRequest.withPosition(setpoint));
    }

    public void setRollerSpeed(double speed) {
        // speed needs to be given in [-1,1]
        rollerMotor.setControl(rollerMotionRequest.withVelocity(speed));
    }

    public boolean atSetpoint(){

        return Math.abs(rollerMotor.getPosition().getValueAsDouble() * 360.0 - currentSetpoint.in(Degrees)) < IntakeConstants.TOLERANCE.in(Degrees);

    }

    // ==================
    // MOTOR ACCESSORS (for FaultMonitor registration)
    // ==================

    public TalonFX getIntakeMotor() {
        return intakeMotor;
    }

    public TalonFX getRollerMotor() {
        return rollerMotor;
    }
}
