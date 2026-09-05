package frc.robot.constants;

public final class OperatorConstants {
    private OperatorConstants() {}

    public static final double STICK_DEADBAND = 0.08;
    /** 1 = linear, 2 = squared. Higher gives finer control near center. */
    public static final double STICK_CURVE_EXPONENT = 2.0;

    /** Turret setpoint change per second at full right-stick deflection. */
    public static final double TURRET_JOYSTICK_RATE_ROT_PER_SEC = 0.25;
    /** Hood setpoint change per second at full left-stick deflection. */
    public static final double HOOD_JOYSTICK_RATE_ROT_PER_SEC = 0.04;

    /** Multiplies the per-loop gyro yaw delta added to the turret setpoint. 0 disables, -1 flips. */
    public static final double TURRET_GYRO_CORRECTION_GAIN = 1.0;

    public static final double HOOD_PRESET_UP_ROT = 0.070;
    public static final double HOOD_PRESET_RIGHT_ROT = 0.055;
    public static final double HOOD_PRESET_LEFT_ROT = 0.040;
    public static final double HOOD_PRESET_DOWN_ROT = 0.005;
}
