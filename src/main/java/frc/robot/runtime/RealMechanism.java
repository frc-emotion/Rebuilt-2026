package frc.robot.runtime;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.runtime.config.MechanismConfig;
import java.util.function.Supplier;

/**
 * Phoenix 6 implementation of one mechanism, configured entirely from a {@link MechanismConfig}.
 * This single class replaces every legacy {@code *IOReal}: the config-apply retry, the boot
 * rotor-zero, the unfused-telemetry vs fused-control CANcoder distinction, and the per-control-type
 * request objects all come from data now.
 */
final class RealMechanism implements MechanismHandle {
  private final MechanismConfig config;
  private final TalonFX motor;
  private final CANcoder encoder; // null when the mechanism has no CANcoder

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Angle> cancoderAbsolute; // null when no CANcoder

  private MechanismState state = MechanismState.kZero;

  RealMechanism(MechanismConfig config) {
    this.config = config;
    CANBus bus = new CANBus(config.bus());
    this.motor = new TalonFX(config.motorId(), bus);

    if (config.encoder().isPresent()) {
      MechanismConfig.Encoder enc = config.encoder().get();
      this.encoder = new CANcoder(enc.id(), bus);
      CANcoderConfiguration encConfig = new CANcoderConfiguration();
      encConfig.MagnetSensor.SensorDirection = enc.sensorDirection();
      encConfig.MagnetSensor.MagnetOffset = enc.magnetOffset();
      applyWithRetry(
          () -> encoder.getConfigurator().apply(encConfig, 0.1), config.name() + " encoder");
    } else {
      this.encoder = null;
    }

    applyWithRetry(
        () -> motor.getConfigurator().apply(buildConfig(), 0.1), config.name() + " motor");

    // Boot rotor-zero (turret straight forward / hood at bottom hard stop at power-on —
    // load-bearing
    // team procedure). RotorSensor + SensorToMechanismRatio converts rotor ticks to output
    // rotations.
    if (config.bootZero()) {
      motor.setPosition(0);
    }

    position = motor.getPosition();
    velocity = motor.getVelocity();
    supplyCurrent = motor.getSupplyCurrent();
    appliedVolts = motor.getMotorVoltage();
    cancoderAbsolute = encoder != null ? encoder.getAbsolutePosition() : null;

    if (encoder != null) {
      ParentDevice.optimizeBusUtilizationForAll(motor, encoder);
    } else {
      motor.optimizeBusUtilization();
    }
    position.setUpdateFrequency(50);
    velocity.setUpdateFrequency(50);
    supplyCurrent.setUpdateFrequency(10);
    appliedVolts.setUpdateFrequency(10);
    if (encoder != null) {
      boolean fusedForControl =
          config
              .feedback()
              .map(f -> f.source() != FeedbackSensorSourceValue.RotorSensor)
              .orElse(false);
      // Fused/remote CANcoder feeds the control loop and needs a fast frame; an unfused telemetry
      // CANcoder is logged only.
      encoder.getPosition().setUpdateFrequency(fusedForControl ? 100 : 4);
      cancoderAbsolute.setUpdateFrequency(4);
    }
  }

  private TalonFXConfiguration buildConfig() {
    TalonFXConfiguration c = new TalonFXConfiguration();
    c.MotorOutput.Inverted = config.inverted();
    c.MotorOutput.NeutralMode = config.neutralMode();

    c.CurrentLimits.StatorCurrentLimitEnable = config.currentLimits().statorEnable();
    c.CurrentLimits.StatorCurrentLimit = config.currentLimits().stator();
    c.CurrentLimits.SupplyCurrentLimitEnable = config.currentLimits().supplyEnable();
    c.CurrentLimits.SupplyCurrentLimit = config.currentLimits().supply();

    c.Slot0.kP = config.gains().kP();
    c.Slot0.kI = config.gains().kI();
    c.Slot0.kD = config.gains().kD();
    c.Slot0.kS = config.gains().kS();
    c.Slot0.kV = config.gains().kV();
    c.Slot0.kA = config.gains().kA();
    c.Slot0.kG = config.gains().kG();

    c.Voltage.PeakForwardVoltage = config.peakForwardVoltage();
    c.Voltage.PeakReverseVoltage = config.peakReverseVoltage();

    config
        .motionMagic()
        .ifPresent(
            mm -> {
              c.MotionMagic.MotionMagicCruiseVelocity = mm.cruiseVelocity();
              c.MotionMagic.MotionMagicAcceleration = mm.acceleration();
              c.MotionMagic.MotionMagicJerk = mm.jerk();
            });

    if (config.feedback().isPresent()) {
      MechanismConfig.Feedback f = config.feedback().get();
      c.Feedback.FeedbackSensorSource = f.source();
      if (f.source() == FeedbackSensorSourceValue.RotorSensor) {
        // Rotor feedback: the whole gear reduction lives in SensorToMechanismRatio.
        c.Feedback.SensorToMechanismRatio = config.gearRatio();
      } else {
        c.Feedback.FeedbackRemoteSensorID = f.remoteSensorId();
        c.Feedback.RotorToSensorRatio = f.rotorToSensorRatio();
        c.Feedback.SensorToMechanismRatio = f.sensorToMechanismRatio();
      }
    } else {
      c.Feedback.SensorToMechanismRatio = config.gearRatio();
    }

    config
        .softLimits()
        .ifPresent(
            s -> {
              c.SoftwareLimitSwitch.ForwardSoftLimitEnable = s.forwardEnable();
              c.SoftwareLimitSwitch.ForwardSoftLimitThreshold = s.forward();
              c.SoftwareLimitSwitch.ReverseSoftLimitEnable = s.reverseEnable();
              c.SoftwareLimitSwitch.ReverseSoftLimitThreshold = s.reverse();
            });

    return c;
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
  public void updateInputs() {
    if (cancoderAbsolute != null) {
      StatusSignal.refreshAll(position, velocity, supplyCurrent, appliedVolts, cancoderAbsolute);
    } else {
      StatusSignal.refreshAll(position, velocity, supplyCurrent, appliedVolts);
    }
    state =
        new MechanismState(
            position.getValueAsDouble(),
            velocity.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            appliedVolts.getValueAsDouble(),
            cancoderAbsolute != null ? cancoderAbsolute.getValueAsDouble() : 0.0,
            motor.getFault_ForwardSoftLimit().getValue(),
            motor.getFault_ReverseSoftLimit().getValue());
  }

  @Override
  public MechanismState state() {
    return state;
  }

  @Override
  public void applyVelocity(double rps) {
    motor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void applyPosition(double rot, double feedforwardVolts) {
    motor.setControl(positionRequest.withPosition(rot).withFeedForward(feedforwardVolts));
  }

  @Override
  public void applyVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void neutral() {
    motor.stopMotor();
  }

  @Override
  public void zero() {
    motor.setPosition(0);
  }

  @Override
  public MechanismConfig config() {
    return config;
  }
}
