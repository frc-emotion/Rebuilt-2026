package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.TimedRobot;

public final class RobotConstants {
    private RobotConstants() {}

    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;

    public static final CANBus MECHANISM_BUS = new CANBus("mechanisms");

    public static final double LOOP_PERIOD_SECONDS = TimedRobot.kDefaultPeriod;

    /** True publishes only CRITICAL telemetry to NetworkTables; log files always get everything. */
    public static final boolean MATCH_MODE = false;
    public static final Logged.Importance NT_MIN_IMPORTANCE =
            MATCH_MODE ? Logged.Importance.CRITICAL : Logged.Importance.DEBUG;

    public static final double DRIVE_MAX_SPEED_MPS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double DRIVE_MAX_ANGULAR_RATE_RAD_PER_SEC = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    public static final double DRIVE_TRANSLATION_DEADBAND_FRACTION = 0.05;
    public static final double DRIVE_ROTATION_DEADBAND_FRACTION = 0.1;

    public record ShotPoint(double distanceMeters, double flywheelRps, double hoodRot) {}

    /** Calibrated 2026-03-17 with distance measured from the old turret camera, not the robot center. */
    public static final ShotPoint[] SHOT_TABLE = {
        new ShotPoint(1.5501340177286882, 41.5, 0.000),
        new ShotPoint(2.47698, 43.5, 0.020),
        new ShotPoint(3.5, 48.5, 0.030),
        new ShotPoint(4.620237, 56.5, 0.035),
    };
}
