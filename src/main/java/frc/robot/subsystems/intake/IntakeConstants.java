package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;

/** Values copied verbatim from legacy IntakeConstants. Tuned numbers are sacred. */
public final class IntakeConstants {
  private IntakeConstants() {}

  public static final int kPivotMotorId = 20;
  public static final int kRollerMotorId = 21;
  public static final int kPivotEncoderId = 22;

  public static final Angle kInAngle = Rotations.of(0.15); // 0.135
  public static final Angle kOutAngle = Rotations.of(0.51); // 0.5

  // Soft limits — prevent motor from commanding past safe range
  public static final double kReverseSoftLimitRot = 0.14; // hard safety wall — past stow target
  public static final double kOvertravelThresholdRot = 0.14; // 0.116 — recovery kicks in past this
  public static final double kForwardSoftLimitRot = 0.515; // past deploy target (forward direction)

  public static final Angle kTolerance = Degrees.of(5); // default (used for stow)
  public static final Angle kDeployTolerance = Degrees.of(15); // looser — rollers start sooner

  // If pivot is more than this past kInAngle (toward deployed), intake is considered "out"
  public static final double kOutThresholdRot = 5.0 / 360.0; // 5 degrees in rotations

  public static final double kRollerVelocityRps = 40;

  // Manual-mode pivot jog scale (new in manual mode; firmware soft limits remain the safety net).
  public static final double kManualPivotVolts = 2.0;

  public static final TalonFXConfiguration kPivotConfig = new TalonFXConfiguration();

  static {
    kPivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kPivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kPivotConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    kPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kPivotConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

    kPivotConfig.Slot0.kG = 0;
    kPivotConfig.Slot0.kS = 0;
    kPivotConfig.Slot0.kV = 0.0;
    kPivotConfig.Slot0.kA = 0.0;
    kPivotConfig.Slot0.kP = 19;
    kPivotConfig.Slot0.kI = 0.0;
    kPivotConfig.Slot0.kD = 0.2; // TODO: tune on robot — resists velocity overshoot at stow

    kPivotConfig.Voltage.PeakForwardVoltage = 10.0;
    kPivotConfig.Voltage.PeakReverseVoltage = -10.0;

    // MotionMagic constraints — prevents slapdown from slamming. TODO: tune on robot.
    kPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 1.5; // 2.0; // RPS
    kPivotConfig.MotionMagic.MotionMagicAcceleration = 2.5; // 4.0;   // RPS^2
    kPivotConfig.MotionMagic.MotionMagicJerk = 40.0; // Smoothing

    kPivotConfig.Feedback.FeedbackRemoteSensorID = kPivotEncoderId;
    kPivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    kPivotConfig.Feedback.RotorToSensorRatio = 27.0; // 27 motor turns per 1 CANcoder turn
    kPivotConfig.Feedback.SensorToMechanismRatio = 1.0; // CANcoder is 1:1 with pivot output

    kPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    kPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kReverseSoftLimitRot;
    kPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    kPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kForwardSoftLimitRot;
  }

  public static final TalonFXConfiguration kRollerConfig = new TalonFXConfiguration();

  static {
    kRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kRollerConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    kRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kRollerConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

    kRollerConfig.Slot0.kS = 0.15;
    kRollerConfig.Slot0.kV = 0.12;
    kRollerConfig.Slot0.kP = 0.3;

    kRollerConfig.Voltage.PeakForwardVoltage = 10.0;
    kRollerConfig.Voltage.PeakReverseVoltage = -10.0;
  }

  public static final double kEncoderOffset = 0.0; // TODO: set magnet offset

  public static final CANcoderConfiguration kEncoderConfig = new CANcoderConfiguration();

  static {
    kEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    kEncoderConfig.MagnetSensor.MagnetOffset = kEncoderOffset;
  }
}
