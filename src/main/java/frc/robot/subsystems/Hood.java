package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/**
 * Hood subsystem with automatic telemetry via Epilogue.
 * 
 * <p>
 * Controls the hood angle for shot trajectory.
 */
@Logged
public class Hood extends SubsystemBase {
    @Logged
    private final TalonFX hoodMotor;

    private final CANcoder hoodEncoder;

    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Current> hoodCurrent;
    private final StatusSignal<Voltage> hoodVoltage;

    private final MotionMagicVoltage hoodMotionRequest;

    private Angle hoodCurrentSetpoint;

    public Hood(CANBus canBus) {
        hoodMotor = new TalonFX(TurretConstants.hoodMotorID, canBus);
        hoodEncoder = new CANcoder(TurretConstants.hoodEncoderID, canBus);

        configureHoodEncoder();
        configureHoodMotor();

        // Cache status signals for telemetry
        hoodVelocity = hoodMotor.getVelocity();
        hoodCurrent = hoodMotor.getSupplyCurrent();
        hoodVoltage = hoodMotor.getMotorVoltage();

        // Motion magic controllers
        hoodMotionRequest = new MotionMagicVoltage(0);

        // Set update frequencies for efficient CAN usage
        hoodVelocity.setUpdateFrequency(50);
        hoodCurrent.setUpdateFrequency(50);
        hoodVoltage.setUpdateFrequency(50);
    }

    private void configureHoodMotor() {
        hoodMotor.getConfigurator().apply(TurretConstants.HOOD_CONFIG);
    }

    private void configureHoodEncoder() {
        hoodEncoder.getConfigurator().apply(TurretConstants.HOOD_ENCODER_CONFIG);
    }

    public void setHoodAngle(Angle setpoint) {
        hoodCurrentSetpoint = setpoint;
        hoodMotor.setControl(hoodMotionRequest.withPosition(setpoint));
    }

    public boolean atHoodSetpoint() {
        return Math.abs(hoodMotor.getPosition().getValueAsDouble()
                - hoodCurrentSetpoint.in(Rotations)) < TurretConstants.hoodTolerance;
    }

    // ==================
    // MOTOR ACCESSORS (for FaultMonitor registration)
    // ==================

    public TalonFX getHoodMotor() {
        return hoodMotor;
    }

    public void stop() {
        hoodMotor.stopMotor();
    }
}
