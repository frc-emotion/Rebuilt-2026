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
    private final NeutralOut rollerNeutralRequest = new NeutralOut();

    private Angle currentSetpoint = Degrees.of(0);

    @Logged(importance = Logged.Importance.CRITICAL) private boolean intakeIsOut = false;
    @Logged(importance = Logged.Importance.DEBUG) private double pivotPositionRot = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double rollerVelocityRPS = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double pivotCurrentAmps = 0.0;

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
        intakeMotor.getSupplyCurrent().setUpdateFrequency(4);      // telemetry
        rollerMotor.getVelocity().setUpdateFrequency(10);          // telemetry
    }

    @Override
    public void periodic() {
        pivotPositionRot = intakeMotor.getPosition().getValueAsDouble();
        intakeIsOut = pivotPositionRot > IntakeConstants.INTAKE_IN_ANGLE.in(Rotations)
                + IntakeConstants.INTAKE_OUT_THRESHOLD_ROT;
        rollerVelocityRPS = rollerMotor.getVelocity().getValueAsDouble();
        pivotCurrentAmps = intakeMotor.getSupplyCurrent().getValueAsDouble();
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
        rollerMotor.setControl(rollerNeutralRequest);
    }

    public boolean atSetpoint(){

        return Math.abs(intakeMotor.getPosition().getValueAsDouble() - currentSetpoint.in(Rotations)) < IntakeConstants.TOLERANCE.in(Rotations);

    }

    // ==================
    // MOTOR ACCESSORS (for FaultMonitor registration)
    // ==================

    public boolean isOut() {
        return pivotPositionRot > IntakeConstants.INTAKE_IN_ANGLE.in(Rotations)
                + IntakeConstants.INTAKE_OUT_THRESHOLD_ROT;
    }

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
