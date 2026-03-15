package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

@Logged
public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor;

    private final VelocityVoltage shooterMotionRequest;

    private double shooterCurrentSetpoint;

    @Logged
    private double shooterVelocityRPS = 0.0;
    @Logged
    private double shooterCurrentAmps = 0.0;
    @Logged
    private double shooterVoltageVolts = 0.0;

    public Shooter(CANBus canBus) {
        shooterMotor = new TalonFX(TurretConstants.shooterMotorID, canBus);

        configureShooterMotor();

        shooterMotionRequest = new VelocityVoltage(0);

        // Disable all default status signals, then enable only what we need
        shooterMotor.optimizeBusUtilization();
        shooterMotor.getVelocity().setUpdateFrequency(50);       // VelocityVoltage feedback + atSetpoint()
        shooterMotor.getSupplyCurrent().setUpdateFrequency(10);  // telemetry
        shooterMotor.getMotorVoltage().setUpdateFrequency(10);   // telemetry
    }

    @Override
    public void periodic() {
        shooterVelocityRPS = shooterMotor.getVelocity().getValueAsDouble();
        shooterCurrentAmps = shooterMotor.getSupplyCurrent().getValueAsDouble();
        shooterVoltageVolts = shooterMotor.getMotorVoltage().getValueAsDouble();
    }

    private void configureShooterMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = shooterMotor.getConfigurator().apply(TurretConstants.SHOOTER_CONFIG, 0.1);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply shooter motor configs: " + status.toString());
        }
    }

    public void setShooterSpeed(AngularVelocity speed) {
        shooterCurrentSetpoint = speed.in(RotationsPerSecond);
        shooterMotor.setControl(shooterMotionRequest.withVelocity(speed.in(RotationsPerSecond)));
    }

    public boolean atShooterSetpoint() {
        return Math.abs(shooterMotor.getVelocity().getValueAsDouble()
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
