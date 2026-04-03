package frc.robot.Constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class IntakeConstants { 
    public static final int intakeMotorID = 20; 
    public static final int rollerMotorID = 21;
    public static final int intakePivotEncoderID = 22;
    public static final double IntakeCurrentSpike = 20;

    public static final Angle INTAKE_IN_ANGLE = Rotations.of(-0.35); 
    public static final Angle INTAKE_OUT_ANGLE = Rotations.of(0.0); 

    // Soft limits — prevent motor from commanding past safe range
    public static final double INTAKE_REVERSE_SOFT_LIMIT = -0.365;    // hard safety wall — past stow target
    public static final double INTAKE_OVERTRAVEL_THRESHOLD = -0.365;  // if past this (toward reverse), recovery kicks in
    public static final double INTAKE_FORWARD_SOFT_LIMIT =  -0.01;    // past deploy target (forward direction)

    public static final Angle TOLERANCE = Degrees.of(5);          // default (used for stow)
    public static final Angle DEPLOY_TOLERANCE = Degrees.of(15);   // looser — rollers start sooner on deploy

    // If pivot is more than this angle away from INTAKE_IN_ANGLE (toward deployed), intake is considered "out"
    public static final double INTAKE_OUT_THRESHOLD_ROT = 5.0 / 360.0; // 5 degrees in rotations


    public static final TalonFXConfiguration INTAKE_CONFIG = new TalonFXConfiguration();

    static{
        INTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        INTAKE_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        INTAKE_CONFIG.CurrentLimits.StatorCurrentLimit = 80.0;
        INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimit = 30.0;

        INTAKE_CONFIG.Slot0.kG = 0;
        INTAKE_CONFIG.Slot0.kS = 0;
        INTAKE_CONFIG.Slot0.kV = 0.0;
        INTAKE_CONFIG.Slot0.kA = 0.0;
        INTAKE_CONFIG.Slot0.kP = 19;
        INTAKE_CONFIG.Slot0.kI = 0.0;
        INTAKE_CONFIG.Slot0.kD = 0.2; // TODO: tune on robot — resists velocity overshoot at stow

        INTAKE_CONFIG.Voltage.PeakForwardVoltage = 10.0;
        INTAKE_CONFIG.Voltage.PeakReverseVoltage = -10.0;

        // MotionMagic constraints — prevents slapdown from slamming. TODO: tune on robot.
        INTAKE_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 1.5;//2.0; // RPS
        INTAKE_CONFIG.MotionMagic.MotionMagicAcceleration = 2.5;//4.0;   // RPS^2
        INTAKE_CONFIG.MotionMagic.MotionMagicJerk = 40.0;          // Smoothing

        INTAKE_CONFIG.Feedback.FeedbackRemoteSensorID = intakePivotEncoderID;
        INTAKE_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        INTAKE_CONFIG.Feedback.RotorToSensorRatio = 27.0; // 27 motor turns per 1 CANcoder turn
        INTAKE_CONFIG.Feedback.SensorToMechanismRatio = 1.0; // CANcoder is 1:1 with pivot output

        // Firmware soft limits — motor cannot command past these positions
        INTAKE_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        INTAKE_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = INTAKE_REVERSE_SOFT_LIMIT;
        INTAKE_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        INTAKE_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = INTAKE_FORWARD_SOFT_LIMIT;
    }

    public static final double INTAKE_ROLLER_VELOCITY = 60;

    public static final TalonFXConfiguration ROLLER_CONFIG = new TalonFXConfiguration();

    static{
        ROLLER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        ROLLER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        ROLLER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        ROLLER_CONFIG.CurrentLimits.StatorCurrentLimit = 60.0;
        ROLLER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        ROLLER_CONFIG.CurrentLimits.SupplyCurrentLimit = 30.0;

        ROLLER_CONFIG.Slot0.kS = 0.15;
        ROLLER_CONFIG.Slot0.kV = 0.12;
        ROLLER_CONFIG.Slot0.kP = 0.3;

        ROLLER_CONFIG.Voltage.PeakForwardVoltage = 10.0;
        ROLLER_CONFIG.Voltage.PeakReverseVoltage = -10.0;
    }

    // ==================
    // INTAKE PIVOT ENCODER CONFIG
    // ==================
    public static final double INTAKE_ENCODER_OFFSET = 0.0; // TODO: set magnet offset

    public static final CANcoderConfiguration INTAKE_ENCODER_CONFIG = new CANcoderConfiguration();

    static {
        INTAKE_ENCODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        INTAKE_ENCODER_CONFIG.MagnetSensor.MagnetOffset = INTAKE_ENCODER_OFFSET;
    }
}
