package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public final class TurretConstants {
    private TurretConstants() {}

    public static final int MOTOR_ID = 51;

    public static final double GEAR_RATIO = 122.0 / 24.0;

    /**
     * Positive turret motion is clockwise viewed from above, and position 0 is wherever the turret sits at boot.
     * The span exceeds one rotation so wrapping always finds a reachable setpoint.
     */
    public static final double FORWARD_LIMIT_ROT = 0.39;
    public static final double REVERSE_LIMIT_ROT = -0.73;

    /** Direction the turret points at position 0, counterclockwise from the intake. It must boot facing straight back. */
    public static final double BOOT_HEADING_ROT = 0.5;

    /** Turret rotation axis from the robot center, +x toward the intake and +y left. Taken from the old robot docs; verify by measuring. */
    public static final Transform2d ROBOT_TO_PIVOT =
            new Transform2d(Units.inchesToMeters(-5.5), Units.inchesToMeters(6.5), Rotation2d.kZero);

    public static final double TOLERANCE_ROT = 0.005;

    public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration();

    static {
        CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
        CONFIG.CurrentLimits.StatorCurrentLimit = 80.0;
        CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        CONFIG.CurrentLimits.SupplyCurrentLimit = 40.0;
        CONFIG.Slot0.kS = 0.2;
        CONFIG.Slot0.kP = 40.0;
        CONFIG.Voltage.PeakForwardVoltage = 10.0;
        CONFIG.Voltage.PeakReverseVoltage = -10.0;
        CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        CONFIG.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        CONFIG.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        CONFIG.MotionMagic.MotionMagicAcceleration = 2.0;
        CONFIG.MotionMagic.MotionMagicJerk = 20.0;
        CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_LIMIT_ROT;
        CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_LIMIT_ROT;
    }
}
