package frc.robot.runtime.config;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import java.util.Optional;

/**
 * One non-drive mechanism motor as DATA (skill-server migration). This is the parsed, validated
 * form of a single entry in {@code src/main/deploy/mechanisms.json}. Every value here was tuned on
 * hardware or in sim and is sacred — the parser copies it verbatim and never rounds or converts it.
 *
 * <p>This record is pure data: it references CTRE signal enums (which carry no native code) so the
 * generic mechanism layer can build a {@code TalonFXConfiguration} from it, but it contains no
 * hardware objects and is safe to construct in a unit test.
 */
public record MechanismConfig(
    String name,
    ControlType controlType,
    int motorId,
    String bus,
    InvertedValue inverted,
    NeutralModeValue neutralMode,
    double gearRatio,
    Gains gains,
    Optional<MotionMagic> motionMagic,
    CurrentLimits currentLimits,
    double peakForwardVoltage,
    double peakReverseVoltage,
    Optional<SoftLimits> softLimits,
    Optional<Feedback> feedback,
    Optional<Encoder> encoder,
    boolean bootZero,
    Optional<Clamp> outputClamp,
    Sim sim) {

  /** The default control mode the interpreter drives this mechanism with. */
  public enum ControlType {
    VELOCITY,
    POSITION,
    VOLTAGE
  }

  public record Gains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagic(double cruiseVelocity, double acceleration, double jerk) {}

  public record CurrentLimits(
      boolean statorEnable, double stator, boolean supplyEnable, double supply) {}

  public record SoftLimits(
      boolean forwardEnable, double forward, boolean reverseEnable, double reverse) {}

  public record Feedback(
      FeedbackSensorSourceValue source,
      int remoteSensorId,
      double rotorToSensorRatio,
      double sensorToMechanismRatio) {}

  public record Encoder(
      int id, SensorDirectionValue sensorDirection, double magnetOffset, boolean fused) {}

  /**
   * An output safety clamp applied to every commanded setpoint (legacy subsystem clamp, as data).
   */
  public record Clamp(double min, double max) {}

  /**
   * Which physics plant + emulated controller the sim mechanism uses (mirrors the legacy IOSim).
   */
  public record Sim(
      Plant plant,
      double moi,
      double gearing,
      double armLengthMeters,
      double armMinRot,
      double armMaxRot,
      double armStartRot,
      boolean gravity,
      Controller controller,
      boolean useKs,
      boolean useFeedforward,
      boolean pinSoftLimits) {

    public enum Plant {
      FLYWHEEL,
      DCMOTOR,
      ARM
    }

    public enum Controller {
      VELOCITY,
      POSITION
    }
  }
}
