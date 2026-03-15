package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import static edu.wpi.first.units.Units.*;

@Logged
public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX rollerMotor;

    private final CANcoder pivotEncoder;

    private final MotionMagicVoltage intakeMotionRequest;
    private final VelocityVoltage rollerMotionRequest;

    private Angle currentSetpoint = Degrees.of(0);
    private boolean deployed = false;

    @Logged
    private double pivotPositionRot = 0.0;
    @Logged
    private double pivotPositionDeg = 0.0;
    @Logged
    private double cancoderAbsoluteRot = 0.0;
    @Logged
    private double rotorPositionRot = 0.0;
    @Logged
    private double pivotVelocityRPS = 0.0;
    @Logged
    private double pivotCurrentAmps = 0.0;
    @Logged
    private double pivotVoltageVolts = 0.0;
    @Logged
    private double rollerVelocityRPS = 0.0;
    @Logged
    private double rollerCurrentAmps = 0.0;
    @Logged
    private double rollerVoltageVolts = 0.0;

    public Intake(CANBus canBus) {
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorID, canBus);
        rollerMotor = new TalonFX(IntakeConstants.rollerMotorID, canBus);
        pivotEncoder = new CANcoder(IntakeConstants.intakePivotEncoderID, canBus);

        configurePivotEncoder();
        configureIntakeMotor();
        configureRollerMotor();

        intakeMotionRequest = new MotionMagicVoltage(currentSetpoint);
        rollerMotionRequest = new VelocityVoltage(0);

        // Disable all default status signals, then enable only what we need
        ParentDevice.optimizeBusUtilizationForAll(intakeMotor, rollerMotor, pivotEncoder);
        intakeMotor.getPosition().setUpdateFrequency(50);          // MotionMagic feedback
        intakeMotor.getVelocity().setUpdateFrequency(10);          // telemetry
        intakeMotor.getSupplyCurrent().setUpdateFrequency(10);     // telemetry
        intakeMotor.getMotorVoltage().setUpdateFrequency(10);      // telemetry
        intakeMotor.getRotorPosition().setUpdateFrequency(10);     // verify gear ratio
        rollerMotor.getVelocity().setUpdateFrequency(10);          // telemetry
        rollerMotor.getSupplyCurrent().setUpdateFrequency(10);     // telemetry
        rollerMotor.getMotorVoltage().setUpdateFrequency(10);      // telemetry
        pivotEncoder.getAbsolutePosition().setUpdateFrequency(10); // encoder health
    }

    @Override
    public void periodic() {
        pivotPositionRot = intakeMotor.getPosition().getValueAsDouble();
        pivotPositionDeg = pivotPositionRot * 360.0;
        cancoderAbsoluteRot = pivotEncoder.getAbsolutePosition().getValueAsDouble();
        rotorPositionRot = intakeMotor.getRotorPosition().getValueAsDouble();
        pivotVelocityRPS = intakeMotor.getVelocity().getValueAsDouble();
        pivotCurrentAmps = intakeMotor.getSupplyCurrent().getValueAsDouble();
        pivotVoltageVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
        rollerVelocityRPS = rollerMotor.getVelocity().getValueAsDouble();
        rollerCurrentAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();
        rollerVoltageVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    }

    private void configurePivotEncoder() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = pivotEncoder.getConfigurator().apply(IntakeConstants.INTAKE_ENCODER_CONFIG, 0.1);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply intake pivot encoder configs: " + status.toString());
        }
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
