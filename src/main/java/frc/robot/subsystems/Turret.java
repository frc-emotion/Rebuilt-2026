package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.Constants.TurretConstants;

/**
 * Turret subsystem with automatic telemetry via Epilogue.
 * 
 * <p>
 * All TalonFX motors are automatically logged with health data.
 * 
 * TODO: Complete implementation with motor configurations and control methods.
 */
@Logged
public class Turret extends SubsystemBase {
    @Logged
    private final TalonFX shooterMotor;
    @Logged
    private final TalonFX hoodMotor;
    @Logged
    private final TalonFX turretMotor;

    private final CANcoder turretEncoder;

    private final StatusSignal<AngularVelocity> turretVelocity;
    private final StatusSignal<Current> turretCurrent;
    private final StatusSignal<Voltage> turretVoltage;
    private final StatusSignal<Angle> turretPosition;

    private final StatusSignal<AngularVelocity> shooterVelocity;
    private final StatusSignal<Current> shooterCurrent;
    private final StatusSignal<Voltage> shooterVoltage;

    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Current> hoodCurrent;
    private final StatusSignal<Voltage> hoodVoltage;

    private final MotionMagicVelocityVoltage shooterMotionRequest;
    private final MotionMagicVoltage turretMotionRequest;
    private final MotionMagicVoltage hoodMotionRequest;

    private double shooterCurrentSetpoint;
    private Angle hoodCurrentSetpoint;
    private Angle turretCurrentSetpoint;    
    
    public Turret(CANBus canBus) {
        shooterMotor = new TalonFX(TurretConstants.shooterMotorID, canBus);
        hoodMotor = new TalonFX(TurretConstants.hoodMotorID, canBus);
        turretMotor = new TalonFX(TurretConstants.turretMotorID, canBus);
        turretEncoder = new CANcoder(TurretConstants.turretEncoderID, canBus);

        configureShooterMotor();
        configureHoodMotor();
        configureTurretEncoder();
        configureTurretMotor();

        // Cache status signals for telemetry
        turretVelocity = turretMotor.getVelocity();
        turretCurrent = turretMotor.getSupplyCurrent();
        turretVoltage = turretMotor.getMotorVoltage();
        turretPosition = turretMotor.getPosition();

        shooterVelocity = shooterMotor.getVelocity();
        shooterCurrent = shooterMotor.getSupplyCurrent();
        shooterVoltage = shooterMotor.getMotorVoltage();

        hoodVelocity = hoodMotor.getVelocity();
        hoodCurrent = hoodMotor.getSupplyCurrent();
        hoodVoltage = hoodMotor.getMotorVoltage();

        // Motion magic controllers
        shooterMotionRequest = new MotionMagicVelocityVoltage(0);
        turretMotionRequest = new MotionMagicVoltage(0);
        hoodMotionRequest = new MotionMagicVoltage(0);

        // Set update frequencies for efficient CAN usage
        turretVelocity.setUpdateFrequency(50);
        turretCurrent.setUpdateFrequency(50);
        turretVoltage.setUpdateFrequency(50);
        turretPosition.setUpdateFrequency(50);

        shooterVelocity.setUpdateFrequency(50);
        shooterCurrent.setUpdateFrequency(50);
        shooterVoltage.setUpdateFrequency(50);

        hoodVelocity.setUpdateFrequency(50);
        hoodCurrent.setUpdateFrequency(50);
        hoodVoltage.setUpdateFrequency(50);
    }

    private void configureTurretMotor() {
        turretMotor.getConfigurator().apply(TurretConstants.TURRET_CONFIG);
    }

    private void configureHoodMotor() {
        hoodMotor.getConfigurator().apply(TurretConstants.HOOD_CONFIG);
    }

    private void configureShooterMotor() {
        shooterMotor.getConfigurator().apply(TurretConstants.SHOOTER_CONFIG);

    }

    private void configureTurretEncoder() {
        turretEncoder.getConfigurator().apply(TurretConstants.TURRET_ENCODER_CONFIG);
    }

    public void setHoodAngle(Angle setpoint) {
        hoodCurrentSetpoint = setpoint;
        hoodMotor.setControl(hoodMotionRequest.withPosition(setpoint));
    }

    public void moveTurret(Angle setpoint) {
        turretCurrentSetpoint = setpoint; 
        turretMotor.setControl(turretMotionRequest.withPosition(setpoint));
    }

    public void setShooterSpeed(double speed) {
        shooterCurrentSetpoint = speed;
        shooterMotor.setControl(shooterMotionRequest.withAcceleration(speed));
    }

    public boolean atShooterSetpoint(){
        return  (shooterMotor.getVelocity().getValueAsDouble() - shooterCurrentSetpoint) < TurretConstants.shooterTolerance;
    }

    public boolean atTurretSetpoint() {
    return Math.abs(turretMotor.getPosition().getValueAsDouble() - turretCurrentSetpoint.in(Rotations)) < TurretConstants.turretTolerance;
}

    public boolean atHoodSetpoint() {
        return Math.abs(hoodMotor.getPosition().getValueAsDouble() - hoodCurrentSetpoint.in(Rotations)) < TurretConstants.hoodTolerance;
    }

    public Rotation2d getTurretPosition() {
       return Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
   }

    // ==================
    // MOTOR ACCESSORS (for FaultMonitor registration)
    // ==================

    public TalonFX getShooterMotor() {
        return shooterMotor;
    }

    public TalonFX getTurretMotor() {
        return turretMotor;
    }

    public TalonFX getHoodMotor() {
        return hoodMotor;
    }

    
    public void stopAll() {
        turretMotor.stopMotor();
        hoodMotor.stopMotor();
        shooterMotor.stopMotor();
    }
}