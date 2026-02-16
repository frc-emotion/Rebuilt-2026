package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
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

    public void setAngle(Angle setpoint) {
        hoodMotor.setControl(hoodMotionRequest.withPosition(setpoint));
    }

    public void moveTurrent(Angle setpoint) {
        turretMotor.setControl(turretMotionRequest.withPosition(setpoint));
    }

    public void setShooterSpeed(double speed) {
        shooterMotor.setControl(shooterMotionRequest.withAcceleration(speed));
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

    // ==================
    // TURRET AIMING METHODS (for vision-based targeting)
    // ==================

    /**
     * Gets the current turret angle from the encoder.
     * 
     * @return Current turret rotation (0 = turret forward matches robot forward)
     */
    public Rotation2d getTurretAngle() {
        turretPosition.refresh();
        return Rotation2d.fromRotations(turretPosition.getValueAsDouble());
    }

    /**
     * Sets the turret to a specific angle using motion magic.
     * 
     * @param angle Target angle (0 = turret forward matches robot forward)
     */
    public void setTurretAngle(Rotation2d angle) {
        turretMotor.setControl(turretMotionRequest.withPosition(angle.getRotations()));
    }

    /**
     * Adjusts the turret by a relative amount (for vision servo control).
     * 
     * @param adjustment Amount to adjust from current position
     */
    public void adjustTurretAngle(Rotation2d adjustment) {
        Rotation2d currentAngle = getTurretAngle();
        Rotation2d newAngle = currentAngle.plus(adjustment);
        setTurretAngle(newAngle);
    }

    /**
     * Sets the hood angle using motion magic.
     * 
     * @param angle Target hood angle
     */
    public void setHoodAngle(Rotation2d angle) {
        hoodMotor.setControl(hoodMotionRequest.withPosition(angle.getRotations()));
    }

    /**
     * Sets the shooter flywheel to a target RPM.
     * 
     * @param rpm Target rotations per minute
     */
    public void setShooterRPM(double rpm) {
        // Convert RPM to rotations per second for the velocity controller
        double rps = rpm / 60.0;
        shooterMotor.setControl(shooterMotionRequest.withVelocity(rps));
    }

    /**
     * Checks if the turret is at the target angle within a tolerance.
     * 
     * @param targetAngle      Target to check against
     * @param toleranceDegrees Acceptable error in degrees
     * @return true if at target
     */
    public boolean isAtTurretAngle(Rotation2d targetAngle, double toleranceDegrees) {
        Rotation2d error = getTurretAngle().minus(targetAngle);
        return Math.abs(error.getDegrees()) <= toleranceDegrees;
    }

    /**
     * Stops all turret motors (for emergency stop or disable).
     */
    public void stopAll() {
        turretMotor.stopMotor();
        hoodMotor.stopMotor();
        shooterMotor.stopMotor();
    }
}