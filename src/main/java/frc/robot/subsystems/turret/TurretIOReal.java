package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.Supplier;

/** Phoenix 6 implementation. All vendor API usage for the turret is confined here. */
public class TurretIOReal implements TurretIO {
  private final TalonFX motor;
  private final CANcoder encoder;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Angle> cancoderAbsolute;

  public TurretIOReal(CANBus bus) {
    motor = new TalonFX(TurretConstants.kMotorId, bus);
    encoder = new CANcoder(TurretConstants.kEncoderId, bus);

    applyWithRetry(
        () -> encoder.getConfigurator().apply(TurretConstants.kEncoderConfig, 0.1),
        "turret encoder");
    applyWithRetry(
        () -> motor.getConfigurator().apply(TurretConstants.kConfig, 0.1), "turret motor");

    // Zero at current position (assumed straight-forward at boot — confirmed team procedure).
    // RotorSensor + SensorToMechanismRatio converts rotor ticks to turret output rotations.
    motor.setPosition(0);

    verifySoftLimitsApplied();

    position = motor.getPosition();
    velocity = motor.getVelocity();
    supplyCurrent = motor.getSupplyCurrent();
    appliedVolts = motor.getMotorVoltage();
    cancoderAbsolute = encoder.getAbsolutePosition();

    ParentDevice.optimizeBusUtilizationForAll(motor, encoder);
    position.setUpdateFrequency(50);
    velocity.setUpdateFrequency(50);
    supplyCurrent.setUpdateFrequency(4);
    appliedVolts.setUpdateFrequency(4);
    cancoderAbsolute.setUpdateFrequency(4);
  }

  private static void applyWithRetry(Supplier<StatusCode> apply, String what) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = apply.get();
      if (status.isOK()) {
        return;
      }
    }
    System.err.println("Could not apply " + what + " configs: " + status.toString());
  }

  // Boot paranoia check (legacy W9): a turret without soft limits destroys its cable chain, and
  // config-apply silently failing after CAN errors is a real CTRE failure mode.
  private void verifySoftLimitsApplied() {
    TalonFXConfiguration readback = new TalonFXConfiguration();
    StatusCode status = motor.getConfigurator().refresh(readback, 0.1);
    if (!status.isOK()) {
      System.err.println("[Turret] WARNING: Could not read back config: " + status);
      return;
    }
    System.out.println("[Turret] Config readback:");
    System.out.println("  FeedbackSource: " + readback.Feedback.FeedbackSensorSource);
    System.out.println("  SensorToMech: " + readback.Feedback.SensorToMechanismRatio);
    System.out.println(
        "  FwdSoftLimit: enabled="
            + readback.SoftwareLimitSwitch.ForwardSoftLimitEnable
            + " threshold="
            + readback.SoftwareLimitSwitch.ForwardSoftLimitThreshold);
    System.out.println(
        "  RevSoftLimit: enabled="
            + readback.SoftwareLimitSwitch.ReverseSoftLimitEnable
            + " threshold="
            + readback.SoftwareLimitSwitch.ReverseSoftLimitThreshold);
  }

  @Override
  public TurretIOInputs updateInputs() {
    StatusSignal.refreshAll(position, velocity, supplyCurrent, appliedVolts, cancoderAbsolute);
    return new TurretIOInputs(
        position.getValueAsDouble(),
        velocity.getValueAsDouble(),
        supplyCurrent.getValueAsDouble(),
        appliedVolts.getValueAsDouble(),
        cancoderAbsolute.getValueAsDouble(),
        motor.getFault_ForwardSoftLimit().getValue(),
        motor.getFault_ReverseSoftLimit().getValue());
  }

  @Override
  public void setTargetPosition(double rot, double feedforwardVolts) {
    motor.setControl(positionRequest.withPosition(rot).withFeedForward(feedforwardVolts));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void zeroAtCurrentPosition() {
    motor.setPosition(0);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
