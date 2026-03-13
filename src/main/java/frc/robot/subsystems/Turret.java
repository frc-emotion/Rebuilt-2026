package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import frc.robot.Constants.TurretConstants;

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

    private Angle turretCurrentSetpoint = Rotations.of(0);

    @Logged
    private double turretPositionRot = 0.0;
    @Logged
    private double turretPositionDegRelative = 0.0;
    @Logged
    private double cancoderAbsoluteRot = 0.0;
    @Logged
    private double rotorPositionRot = 0.0;
    @Logged
    private boolean faultRemoteSensorInvalid = false;
    @Logged
    private boolean faultFusedOutOfSync = false;
    @Logged
    private boolean faultForwardSoftLimit = false;
    @Logged
    private boolean faultReverseSoftLimit = false;

    public Turret(CANBus canBus) {
        turretMotor = new TalonFX(TurretConstants.turretMotorID, canBus);
        turretEncoder = new CANcoder(TurretConstants.turretEncoderID, canBus);

        configureTurretEncoder();
        configureTurretMotor();

        // FusedCANcoder handles absolute position seeding from the CANcoder
        // automatically — no manual setPosition() needed.
        // CANcoder is 1:1 with rotor (RotorToSensorRatio = 1.0).
        // SensorToMechanismRatio = 5.08 converts CANcoder rotations to
        // turret output rotations. motor.getPosition() reports in turret rotations.

        verifySoftLimitsApplied();

        turretVelocity = turretMotor.getVelocity();
        turretCurrent = turretMotor.getSupplyCurrent();
        turretVoltage = turretMotor.getMotorVoltage();
        turretPosition = turretMotor.getPosition();

        turretMotionRequest = new MotionMagicVoltage(0);
        manualTurretRequest = new VoltageOut(0);

        turretVelocity.setUpdateFrequency(50);
        turretCurrent.setUpdateFrequency(50);
        turretVoltage.setUpdateFrequency(50);
        turretPosition.setUpdateFrequency(50);
    }

    private void configureTurretMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = turretMotor.getConfigurator().apply(TurretConstants.TURRET_CONFIG, 0.1);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply turret motor configs: " + status.toString());
        }
    }

    private void configureTurretEncoder() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = turretEncoder.getConfigurator().apply(TurretConstants.TURRET_ENCODER_CONFIG, 0.1);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply turret encoder configs: " + status.toString());
        }
    }

    public void moveTurret(Angle setpoint) {
        turretCurrentSetpoint = setpoint;
        turretMotor.setControl(turretMotionRequest.withPosition(turretCurrentSetpoint));
    }

    public void moveTurret(Angle setpoint, double feedforwardVolts) {
        turretCurrentSetpoint = setpoint;
        turretMotor.setControl(turretMotionRequest.withPosition(turretCurrentSetpoint).withFeedForward(feedforwardVolts));
    }

    public void setTurretVoltage(double joystickInput) {
        double voltage = joystickInput * 3.0;
        turretMotor.setControl(manualTurretRequest.withOutput(voltage));
    }

    public boolean atTurretSetpoint() {
        return Math.abs(turretMotor.getPosition().getValueAsDouble()
                - turretCurrentSetpoint.in(Rotations)) < TurretConstants.turretTolerance;
    }

    public Rotation2d getTurretPosition() {
        return Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        turretPositionRot = turretMotor.getPosition().getValueAsDouble();
        turretPositionDegRelative = (turretPositionRot - TurretConstants.TURRET_FORWARD_POSITION) * 360.0;
        cancoderAbsoluteRot = turretEncoder.getAbsolutePosition().getValueAsDouble();
        rotorPositionRot = turretMotor.getRotorPosition().getValueAsDouble();
        faultRemoteSensorInvalid = turretMotor.getFault_RemoteSensorDataInvalid().getValue();
        faultFusedOutOfSync = turretMotor.getFault_FusedSensorOutOfSync().getValue();
        faultForwardSoftLimit = turretMotor.getFault_ForwardSoftLimit().getValue();
        faultReverseSoftLimit = turretMotor.getFault_ReverseSoftLimit().getValue();
    }

    // ==================
    // MOTOR ACCESSORS (for FaultMonitor registration)
    // ==================

    public TalonFX getTurretMotor() {
        return turretMotor;
    }

    /**
     * Reads back the motor's actual applied config and prints soft limit status
     * to console. If this doesn't match what we sent, the config didn't apply.
     */
    private void verifySoftLimitsApplied() {
        var readback = new TalonFXConfiguration();
        var status = turretMotor.getConfigurator().refresh(readback, 0.1);
        if (status.isOK()) {
            System.out.println("[Turret] Config readback:");
            System.out.println("  FeedbackSource: " + readback.Feedback.FeedbackSensorSource);
            System.out.println("  RemoteSensorID: " + readback.Feedback.FeedbackRemoteSensorID);
            System.out.println("  RotorToSensor: " + readback.Feedback.RotorToSensorRatio);
            System.out.println("  SensorToMech: " + readback.Feedback.SensorToMechanismRatio);
            System.out.println("  FwdSoftLimit: enabled=" + readback.SoftwareLimitSwitch.ForwardSoftLimitEnable
                    + " threshold=" + readback.SoftwareLimitSwitch.ForwardSoftLimitThreshold);
            System.out.println("  RevSoftLimit: enabled=" + readback.SoftwareLimitSwitch.ReverseSoftLimitEnable
                    + " threshold=" + readback.SoftwareLimitSwitch.ReverseSoftLimitThreshold);
        } else {
            System.err.println("[Turret] WARNING: Could not read back config: " + status);
        }
    }

    public void stop() {
        turretMotor.stopMotor();
    }
}