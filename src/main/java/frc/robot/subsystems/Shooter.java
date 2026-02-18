package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/**
 * Shooter subsystem with automatic telemetry via Epilogue.
 * 
 * <p>
 * Controls the flywheel motor for shooting balls.
 */
@Logged
public class Shooter extends SubsystemBase {
    @Logged
    private final TalonFX shooterMotor;

    private final StatusSignal<AngularVelocity> shooterVelocity;
    private final StatusSignal<Current> shooterCurrent;
    private final StatusSignal<Voltage> shooterVoltage;

    private final VelocityVoltage shooterMotionRequest;

    private double shooterCurrentSetpoint;

    public Shooter(CANBus canBus) {
        shooterMotor = new TalonFX(TurretConstants.shooterMotorID, canBus);

        configureShooterMotor();

        // Cache status signals for telemetry
        shooterVelocity = shooterMotor.getVelocity();
        shooterCurrent = shooterMotor.getSupplyCurrent();
        shooterVoltage = shooterMotor.getMotorVoltage();

        // Motion magic controllers
        shooterMotionRequest = new VelocityVoltage(0);

        // Set update frequencies for efficient CAN usage
        shooterVelocity.setUpdateFrequency(50);
        shooterCurrent.setUpdateFrequency(50);
        shooterVoltage.setUpdateFrequency(50);
    }

    private void configureShooterMotor() {
        shooterMotor.getConfigurator().apply(TurretConstants.SHOOTER_CONFIG);

    }

    public void setShooterSpeed(AngularVelocity speed) {
        shooterCurrentSetpoint = speed.in(RotationsPerSecond);
        shooterMotor.setControl(shooterMotionRequest.withVelocity(speed.in(RotationsPerSecond)));
    }

    public boolean atShooterSetpoint() {
        return (shooterMotor.getVelocity().getValueAsDouble()
                - shooterCurrentSetpoint) < TurretConstants.shooterTolerance;
    }

    // ==================
    // MOTOR ACCESSORS (for FaultMonitor registration)
    // ==================

    public TalonFX getShooterMotor() {
        return shooterMotor;
    }

    public void stop() {
        shooterMotor.stopMotor();
    }
}
