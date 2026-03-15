package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
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

@Logged
public class Hood extends SubsystemBase {
    private final TalonFX hoodMotor;

    private final CANcoder hoodEncoder;

    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Current> hoodCurrent;
    private final StatusSignal<Voltage> hoodVoltage;

    private final MotionMagicVoltage hoodMotionRequest;
    private final com.ctre.phoenix6.controls.VoltageOut hoodManualVoltageRequest = new com.ctre.phoenix6.controls.VoltageOut(
            0);

    private Angle hoodCurrentSetpoint = Rotations.of(0);

    @Logged
    private double hoodPositionRot = 0.0;

    public Hood(CANBus canBus) {
        hoodMotor = new TalonFX(TurretConstants.hoodMotorID, canBus);
        hoodEncoder = new CANcoder(TurretConstants.hoodEncoderID, canBus);

        configureHoodEncoder();
        configureHoodMotor();

        hoodVelocity = hoodMotor.getVelocity();
        hoodCurrent = hoodMotor.getSupplyCurrent();
        hoodVoltage = hoodMotor.getMotorVoltage();

        hoodMotionRequest = new MotionMagicVoltage(0);

        hoodVelocity.setUpdateFrequency(50);
        hoodCurrent.setUpdateFrequency(50);
        hoodVoltage.setUpdateFrequency(50);
    }

    private void configureHoodMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = hoodMotor.getConfigurator().apply(TurretConstants.HOOD_CONFIG, 0.1);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply hood motor configs: " + status.toString());
        }
    }

    private void configureHoodEncoder() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = hoodEncoder.getConfigurator().apply(TurretConstants.HOOD_ENCODER_CONFIG, 0.1);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply hood encoder configs: " + status.toString());
        }
    }

    public void setHoodAngle(Angle setpoint) {
        hoodCurrentSetpoint = setpoint;
        hoodMotor.setControl(hoodMotionRequest.withPosition(setpoint));
    }

    public boolean atHoodSetpoint() {
        return Math.abs(hoodMotor.getPosition().getValueAsDouble()
                - hoodCurrentSetpoint.in(Rotations)) < TurretConstants.hoodTolerance;
    }

    @Override
    public void periodic() {
        hoodPositionRot = hoodMotor.getPosition().getValueAsDouble();
    }

    public TalonFX getHoodMotor() {
        return hoodMotor;
    }

    public void setHoodManualVoltage(double joystickInput) {
        double manualVolts = joystickInput * 3.0;
        hoodMotor.setControl(hoodManualVoltageRequest.withOutput(manualVolts));
    }

    public void stop() {
        hoodMotor.stopMotor();
    }
}
