package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;

/** Phoenix 6 implementation. All vendor API usage for the indexer is confined here. */
public class IndexerIOReal implements IndexerIO {
  private final TalonFX horizontalMotor;
  private final TalonFX verticalMotor;
  private final TalonFX upwardMotor;

  private final VelocityVoltage horizontalRequest = new VelocityVoltage(0);
  private final VelocityVoltage verticalRequest = new VelocityVoltage(0);
  private final VelocityVoltage upwardRequest = new VelocityVoltage(0);

  private final StatusSignal<AngularVelocity> horizontalVelocity;
  private final StatusSignal<AngularVelocity> verticalVelocity;
  private final StatusSignal<AngularVelocity> upwardVelocity;

  public IndexerIOReal(CANBus bus) {
    horizontalMotor = new TalonFX(IndexerConstants.kHorizontalMotorId, bus);
    verticalMotor = new TalonFX(IndexerConstants.kVerticalMotorId, bus);
    upwardMotor = new TalonFX(IndexerConstants.kUpwardMotorId, bus);

    applyWithRetry(horizontalMotor, IndexerConstants.kHorizontalConfig, "horizontal indexer");
    applyWithRetry(verticalMotor, IndexerConstants.kVerticalConfig, "vertical indexer");
    applyWithRetry(upwardMotor, IndexerConstants.kUpwardConfig, "upward indexer");

    horizontalVelocity = horizontalMotor.getVelocity();
    verticalVelocity = verticalMotor.getVelocity();
    upwardVelocity = upwardMotor.getVelocity();

    ParentDevice.optimizeBusUtilizationForAll(horizontalMotor, verticalMotor, upwardMotor);
    horizontalVelocity.setUpdateFrequency(10);
    verticalVelocity.setUpdateFrequency(10);
    upwardVelocity.setUpdateFrequency(10);
  }

  private static void applyWithRetry(TalonFX motor, TalonFXConfiguration config, String what) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(config, 0.1);
      if (status.isOK()) {
        return;
      }
    }
    System.err.println("Could not apply " + what + " configs: " + status.toString());
  }

  @Override
  public IndexerIOInputs updateInputs() {
    StatusSignal.refreshAll(horizontalVelocity, verticalVelocity, upwardVelocity);
    return new IndexerIOInputs(
        horizontalVelocity.getValueAsDouble(),
        verticalVelocity.getValueAsDouble(),
        upwardVelocity.getValueAsDouble());
  }

  @Override
  public void setVelocity(Stage stage, double rps) {
    switch (stage) {
      case HORIZONTAL -> horizontalMotor.setControl(horizontalRequest.withVelocity(rps));
      case VERTICAL -> verticalMotor.setControl(verticalRequest.withVelocity(rps));
      case UPWARD -> upwardMotor.setControl(upwardRequest.withVelocity(rps));
    }
  }

  @Override
  public void stop(Stage stage) {
    switch (stage) {
      case HORIZONTAL -> horizontalMotor.stopMotor();
      case VERTICAL -> verticalMotor.stopMotor();
      case UPWARD -> upwardMotor.stopMotor();
    }
  }
}
