package frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

public final class ClimbConstants {
    // IDs - Defined locally here to match subsystem style, mirrored in CANID for documentation
    public static final int CLIMB_LEADER_ID = 40;
    public static final int CLIMB_FOLLOWER_ID = 41;

    // Mechanics
    public static final double GEAR_RATIO = 12.0; 
    
    // TODO: Measure how many motor rotations it takes to reach full height
    // Example: If spool is 2 inches diameter -> Circumference ~6.28 inches.
    // If max height is 25 inches -> ~4 spool turns.
    // With 12:1 gear ratio -> 4 * 12 = 48 motor rotations.
    public static final double MAX_HEIGHT_ROTATIONS = 50.0; 
    public static final double THREE_QUARTER_HEIGHT = MAX_HEIGHT_ROTATIONS * 0.75;
    public static final double BOTTOM_HEIGHT = 0.5; // Keep a small buffer from physical hardstop

    public static final TalonFXConfiguration CLIMB_CONFIG = new TalonFXConfiguration();

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
        CLIMB_CONFIG.Slot0.kP = 40.0; 
        CLIMB_CONFIG.Slot0.kI = 0.0;
        CLIMB_CONFIG.Slot0.kD = 2.0;
        CLIMB_CONFIG.Slot0.kV = 0.12; 
        CLIMB_CONFIG.Slot0.kG = 0.5;  
        CLIMB_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static; 

        // Motion Magic constraints
        CLIMB_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 80.0; // RPS
        CLIMB_CONFIG.MotionMagic.MotionMagicAcceleration = 160.0; // RPS^2
        CLIMB_CONFIG.MotionMagic.MotionMagicJerk = 1600.0; // Smoothing
    }
}