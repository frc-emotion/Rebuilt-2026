package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Phoenix 6 implementation. All vendor API usage for the shooter is confined here. */
public class ShooterIOReal implements ShooterIO {
  private final TalonFX motor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Voltage> appliedVolts;

  public ShooterIOReal(CANBus bus) {
    motor = new TalonFX(ShooterConstants.kMotorId, bus);

    // CAN devices are sometimes not ready at boot; retry the config apply (legacy pattern).
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(ShooterConstants.kConfig, 0.1);
      if (status.isOK()) {
        break;
      }
    }
    if (!status.isOK()) {
      System.err.println("Could not apply shooter motor configs: " + status.toString());
    }

    velocity = motor.getVelocity();
    supplyCurrent = motor.getSupplyCurrent();
    appliedVolts = motor.getMotorVoltage();

    motor.optimizeBusUtilization();
    velocity.setUpdateFrequency(50);
    supplyCurrent.setUpdateFrequency(10);
    appliedVolts.setUpdateFrequency(10);
  }

  @Override
  public ShooterIOInputs updateInputs() {
    StatusSignal.refreshAll(velocity, supplyCurrent, appliedVolts);
    return new ShooterIOInputs(
        velocity.getValueAsDouble(),
        supplyCurrent.getValueAsDouble(),
        appliedVolts.getValueAsDouble());
  }

  @Override
  public void setVelocity(double rps) {
    motor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
