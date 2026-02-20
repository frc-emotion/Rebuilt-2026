package frc.robot.Constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

public final class ClimbConstants {
    // IDs - Defined locally here to match subsystem style, mirrored in CANID for documentation
    public static final int CLIMB_LEADER_ID = 40;
    public static final int CLIMB_FOLLOWER_ID = 41;

    public static final double CLIMB_GEAR_RATIO = 1.0;

    public static final int CLIMB_ENCODER_ID = 67;

    // Mechanics
    public static final double GEAR_RATIO = 12.0; 

    public static final double TOLERANCE = 0.5;

    public static final double manual_kG = 0.5;

    public static final TalonFXConfiguration CLIMB_CONFIG = new TalonFXConfiguration();

    public static enum ClimbLevel {
        LEVEL_1(0.0, 0.0),
        LEVEL_2(0.0, 0.0),
        LEVEL_3(0.0, 0.0);

        public final double inner;
        public final double outer;

        ClimbLevel(double inner, double outer) {
            this.inner = inner;
            this.outer = outer;
        }
    }

    public static final double MAX_HEIGHT_ROTATIONS = 1000;

    static {
        // Motor Configuration
        CLIMB_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        CLIMB_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Crucial for climbing!

        // Current Limits - Krakens are powerful, protect the mechanism
        CLIMB_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        CLIMB_CONFIG.CurrentLimits.StatorCurrentLimit = 80.0;
        CLIMB_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        CLIMB_CONFIG.CurrentLimits.SupplyCurrentLimit = 60.0;

        // Feedback
        CLIMB_CONFIG.Feedback.SensorToMechanismRatio = 1.0; 

        // Soft Limits (Safety)
        CLIMB_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_HEIGHT_ROTATIONS;
        CLIMB_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        CLIMB_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        CLIMB_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // Motion Magic PID Slots
        // Slot 0: Position Control
        CLIMB_CONFIG.Slot0.kP = 0.1; 
        CLIMB_CONFIG.Slot0.kI = 0.0;
        CLIMB_CONFIG.Slot0.kD = 0.0;
        CLIMB_CONFIG.Slot0.kV = 0.0; 
        CLIMB_CONFIG.Slot0.kG = 0.5;  
        CLIMB_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static; 

        // Motion Magic constraints
        // CLIMB_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 80.0; // RPS
        // CLIMB_CONFIG.MotionMagic.MotionMagicAcceleration = 160.0; // RPS^2
        // CLIMB_CONFIG.MotionMagic.MotionMagicJerk = 1600.0; // Smoothing

        CLIMB_CONFIG.Feedback.FeedbackRemoteSensorID = CLIMB_ENCODER_ID;
        CLIMB_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        CLIMB_CONFIG.Feedback.SensorToMechanismRatio = CLIMB_GEAR_RATIO;
        CLIMB_CONFIG.Feedback.RotorToSensorRatio = 1.0;
    }

    public static CANcoderConfiguration CLIMB_ENCODER_CONFIG = new CANcoderConfiguration(); 

    public static double CLIMB_ENCODER_OFFSET = 0;

    static {

        CLIMB_ENCODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CLIMB_ENCODER_CONFIG.MagnetSensor.MagnetOffset = CLIMB_ENCODER_OFFSET;
    }
}
