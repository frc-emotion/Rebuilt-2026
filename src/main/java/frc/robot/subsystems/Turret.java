package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

@Logged
public class Turret extends SubsystemBase {
    private final TalonFX turretMotor;

    private final CANcoder turretEncoder;

    private final MotionMagicVoltage turretMotionRequest;
    private final VoltageOut manualTurretRequest;

    private Angle turretCurrentSetpoint = Rotations.of(0);

    @Logged
    private double turretSetpointRot = 0.0;
    @Logged
    private double turretPositionRot = 0.0;
    @Logged
    private double turretPositionDegRelative = 0.0;
    @Logged
    private double turretErrorRot = 0.0;
    @Logged
    private double cancoderAbsoluteRot = 0.0;
    @Logged
    private double rotorPositionRot = 0.0;
    @Logged
    private boolean faultForwardSoftLimit = false;
    @Logged
    private boolean faultReverseSoftLimit = false;
    @Logged
    private double turretVelocityRPS = 0.0;
    @Logged
    private double turretCurrentAmps = 0.0;
    @Logged
    private double turretVoltageVolts = 0.0;

    public Turret(CANBus canBus) {
        turretMotor = new TalonFX(TurretConstants.turretMotorID, canBus);
        turretEncoder = new CANcoder(TurretConstants.turretEncoderID, canBus);

        configureTurretEncoder();
        configureTurretMotor();

        // Zero at current position (assumed straight-forward at boot, like hood).
        // RotorSensor + SensorToMechanismRatio = 5.08 converts rotor ticks
        // to turret output rotations.  motor.getPosition() reports in turret rotations.
        turretMotor.setPosition(0);

        verifySoftLimitsApplied();

        turretMotionRequest = new MotionMagicVoltage(0);
        manualTurretRequest = new VoltageOut(0);

        // Disable all default status signals, then enable only what we need
        ParentDevice.optimizeBusUtilizationForAll(turretMotor, turretEncoder);
        turretMotor.getPosition().setUpdateFrequency(50);         // MotionMagic feedback
        turretMotor.getVelocity().setUpdateFrequency(10);         // telemetry
        turretMotor.getSupplyCurrent().setUpdateFrequency(10);    // telemetry
        turretMotor.getMotorVoltage().setUpdateFrequency(10);     // telemetry
        turretMotor.getRotorPosition().setUpdateFrequency(10);    // verify FusedCANcoder
        turretEncoder.getAbsolutePosition().setUpdateFrequency(10); // encoder health
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
        double clampedRot = MathUtil.clamp(setpoint.in(Rotations),
                TurretConstants.TURRET_REVERSE_LIMIT, TurretConstants.TURRET_FORWARD_LIMIT);
        turretCurrentSetpoint = Rotations.of(clampedRot);
        turretMotor.setControl(turretMotionRequest.withPosition(turretCurrentSetpoint));
    }

    public void moveTurret(Angle setpoint, double feedforwardVolts) {
        double clampedRot = MathUtil.clamp(setpoint.in(Rotations),
                TurretConstants.TURRET_REVERSE_LIMIT, TurretConstants.TURRET_FORWARD_LIMIT);
        turretCurrentSetpoint = Rotations.of(clampedRot);
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
        turretSetpointRot = turretCurrentSetpoint.in(Rotations);
        turretErrorRot = turretSetpointRot - turretPositionRot;
        turretPositionDegRelative = turretPositionRot * 360.0;
        cancoderAbsoluteRot = turretEncoder.getAbsolutePosition().getValueAsDouble();
        rotorPositionRot = turretMotor.getRotorPosition().getValueAsDouble();
        faultForwardSoftLimit = turretMotor.getFault_ForwardSoftLimit().getValue();
        faultReverseSoftLimit = turretMotor.getFault_ReverseSoftLimit().getValue();
        turretVelocityRPS = turretMotor.getVelocity().getValueAsDouble();
        turretCurrentAmps = turretMotor.getSupplyCurrent().getValueAsDouble();
        turretVoltageVolts = turretMotor.getMotorVoltage().getValueAsDouble();
    }

    // ==================
    // MOTOR ACCESSORS (for FaultMonitor registration)
    // ==================

    public TalonFX getTurretMotor() {
        return turretMotor;
    }

    private void verifySoftLimitsApplied() {
        var readback = new TalonFXConfiguration();
        var status = turretMotor.getConfigurator().refresh(readback, 0.1);
        if (status.isOK()) {
            System.out.println("[Turret] Config readback:");
            System.out.println("  FeedbackSource: " + readback.Feedback.FeedbackSensorSource);
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