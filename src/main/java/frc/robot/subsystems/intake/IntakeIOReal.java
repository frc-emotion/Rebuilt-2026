package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import java.util.function.Supplier;

/** Phoenix 6 implementation. All vendor API usage for the intake is confined here. */
public class IntakeIOReal implements IntakeIO {
  private final TalonFX pivotMotor;
  private final TalonFX rollerMotor;
  private final CANcoder pivotEncoder;

  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);
  private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
  private final VelocityVoltage rollerRequest = new VelocityVoltage(0);
  private final NeutralOut rollerNeutralRequest = new NeutralOut();

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<AngularVelocity> rollerVelocity;

  public IntakeIOReal(CANBus bus) {
    pivotMotor = new TalonFX(IntakeConstants.kPivotMotorId, bus);
    rollerMotor = new TalonFX(IntakeConstants.kRollerMotorId, bus);
    pivotEncoder = new CANcoder(IntakeConstants.kPivotEncoderId, bus);

    applyWithRetry(
        () -> pivotEncoder.getConfigurator().apply(IntakeConstants.kEncoderConfig, 0.1),
        "intake pivot encoder");
    applyWithRetry(
        () -> pivotMotor.getConfigurator().apply(IntakeConstants.kPivotConfig, 0.1), "intake motor");
    applyWithRetry(
        () -> rollerMotor.getConfigurator().apply(IntakeConstants.kRollerConfig, 0.1),
        "roller motor");

    pivotPosition = pivotMotor.getPosition();
    pivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    rollerVelocity = rollerMotor.getVelocity();

    ParentDevice.optimizeBusUtilizationForAll(pivotMotor, rollerMotor, pivotEncoder);
    // RemoteCANcoder feedback source — the TalonFX needs this signal to read its remote sensor.
    pivotEncoder.getPosition().setUpdateFrequency(100);
    pivotPosition.setUpdateFrequency(50);
    pivotSupplyCurrent.setUpdateFrequency(4);
    rollerVelocity.setUpdateFrequency(10);
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

  @Override
  public IntakeIOInputs updateInputs() {
    StatusSignal.refreshAll(pivotPosition, pivotSupplyCurrent, rollerVelocity);
    return new IntakeIOInputs(
        pivotPosition.getValueAsDouble(),
        pivotSupplyCurrent.getValueAsDouble(),
        rollerVelocity.getValueAsDouble());
  }

  @Override
  public void setPivotTarget(double rot) {
    pivotMotor.setControl(pivotRequest.withPosition(rot));
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotMotor.setControl(pivotVoltageRequest.withOutput(volts));
  }

  @Override
  public void setRollerVelocity(double rps) {
    rollerMotor.setControl(rollerRequest.withVelocity(rps));
  }

  @Override
  public void stopRoller() {
    rollerMotor.setControl(rollerNeutralRequest);
  }

  @Override
  public void stop() {
    pivotMotor.stopMotor();
    rollerMotor.stopMotor();
  }
}
