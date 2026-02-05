package frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ClimbConstants {

    // Don't know the actual IDs have some random numbers
    public static final int climbMotorLeaderID = 11;
    public static final int climbMotorFollowerID = 12;

    public static final double CLIMB_UP_SPEED = 0.5; // IDK SPEED SO I JUS PUT 0.5
    public static final double CLIMB_DOWN_SPEED = -0.5; // IDK SPEED SO I JUS PUT -0.5

    public static final TalonFXConfiguration CLIMB_CONFIG = new TalonFXConfiguration();

    static {
        CLIMB_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        CLIMB_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // TODO: Determine actual mechanism ratio
        CLIMB_CONFIG.Feedback.SensorToMechanismRatio = 12.0;

        // TODO: We need to determine the actual soft limits

        // CLIMB_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // CLIMB_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0; // Max
        // height
        // CLIMB_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // CLIMB_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0; // Min
        // height

        // TODO: Determine actual PID constants
        CLIMB_CONFIG.Slot0.kG = 0.0;
        CLIMB_CONFIG.Slot0.kS = 0.0;
        CLIMB_CONFIG.Slot0.kV = 0.0;
        CLIMB_CONFIG.Slot0.kA = 0.0;
        CLIMB_CONFIG.Slot0.kP = 0.0;
        CLIMB_CONFIG.Slot0.kI = 0.0;
        CLIMB_CONFIG.Slot0.kD = 0.0;
    }
}
