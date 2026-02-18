package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.TurretConstants;

/**
 * Turret subsystem with automatic telemetry via Epilogue.
 * 
 * <p>
 * Controls the turret rotation for aiming.
 */
@Logged
public class Turret extends SubsystemBase {
    @Logged
    private final TalonFX turretMotor;

    private final CANcoder turretEncoder;

    private final StatusSignal<AngularVelocity> turretVelocity;
    private final StatusSignal<Current> turretCurrent;
    private final StatusSignal<Voltage> turretVoltage;
    private final StatusSignal<Angle> turretPosition;

    private final MotionMagicVoltage turretMotionRequest;
    private final VoltageOut manualTurretRequest;

    private Angle turretCurrentSetpoint;

    public Turret(CANBus canBus) {
        turretMotor = new TalonFX(TurretConstants.turretMotorID, canBus);
        turretEncoder = new CANcoder(TurretConstants.turretEncoderID, canBus);

        configureTurretEncoder();
        configureTurretMotor();

        // Cache status signals for telemetry
        turretVelocity = turretMotor.getVelocity();
        turretCurrent = turretMotor.getSupplyCurrent();
        turretVoltage = turretMotor.getMotorVoltage();
        turretPosition = turretMotor.getPosition();

        // Motion magic controllers
        turretMotionRequest = new MotionMagicVoltage(0);
        manualTurretRequest = new VoltageOut(0);

        // Set update frequencies for efficient CAN usage
        turretVelocity.setUpdateFrequency(50);
        turretCurrent.setUpdateFrequency(50);
        turretVoltage.setUpdateFrequency(50);
        turretPosition.setUpdateFrequency(50);
    }

    private void configureTurretMotor() {
        turretMotor.getConfigurator().apply(TurretConstants.TURRET_CONFIG);
    }

    private void configureTurretEncoder() {
        turretEncoder.getConfigurator().apply(TurretConstants.TURRET_ENCODER_CONFIG);
    }

    public void moveTurret(Angle setpoint) {
        turretCurrentSetpoint = setpoint;
        turretMotor.setControl(turretMotionRequest.withPosition(setpoint));
    }

    public void setTurretVoltage(double joystickInput) {
        double turretVoltage = joystickInput * 12.0;
        turretMotor.setControl(manualTurretRequest.withOutput(turretVoltage));

    }

    public boolean atTurretSetpoint() {
        return Math.abs(turretMotor.getPosition().getValueAsDouble()
                - turretCurrentSetpoint.in(Rotations)) < TurretConstants.turretTolerance;
    }

    public Rotation2d getTurretPosition() {
        return Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
    }

    // ==================
    // MOTOR ACCESSORS (for FaultMonitor registration)
    // ==================

    public TalonFX getTurretMotor() {
        return turretMotor;
    }

    public void stop() {
        turretMotor.stopMotor();
    }
}