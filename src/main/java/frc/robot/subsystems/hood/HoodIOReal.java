package frc.robot.subsystems.hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Phoenix 6 implementation. All vendor API usage for the hood is confined here. */
public class HoodIOReal implements HoodIO {
  private final TalonFX motor;
  private final CANcoder encoder;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Voltage> appliedVolts;

  public HoodIOReal(CANBus bus) {
    motor = new TalonFX(HoodConstants.kMotorId, bus);
    encoder = new CANcoder(HoodConstants.kEncoderId, bus);

    applyWithRetry(
        () -> encoder.getConfigurator().apply(HoodConstants.kEncoderConfig, 0.1), "hood encoder");
    applyWithRetry(() -> motor.getConfigurator().apply(HoodConstants.kConfig, 0.1), "hood motor");

    // Zero hood at current position (assumed to be bottom/home on startup — confirmed team
    // procedure). Feedback is RotorSensor; the CANcoder is configured but unfused, as legacy.
    motor.setPosition(0);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    supplyCurrent = motor.getSupplyCurrent();
    appliedVolts = motor.getMotorVoltage();

    ParentDevice.optimizeBusUtilizationForAll(motor, encoder);
    position.setUpdateFrequency(50);
    velocity.setUpdateFrequency(4);
    supplyCurrent.setUpdateFrequency(4);
    appliedVolts.setUpdateFrequency(4);
  }

  private static void applyWithRetry(java.util.function.Supplier<StatusCode> apply, String what) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = apply.get();
      if (status.isOK()) {
        return;
      }
    }
    System.err.println("Could not apply " + what + " configs: " + status.toString());
  }

  @Override
  public HoodIOInputs updateInputs() {
    StatusSignal.refreshAll(position, velocity, supplyCurrent, appliedVolts);
    return new HoodIOInputs(
        position.getValueAsDouble(),
        velocity.getValueAsDouble(),
        supplyCurrent.getValueAsDouble(),
        appliedVolts.getValueAsDouble());
  }

  @Override
  public void setTargetPosition(double rot) {
    motor.setControl(positionRequest.withPosition(rot));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
