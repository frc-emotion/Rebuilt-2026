package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.RobotConstants;

public final class ControllerUtil {
    
    private ControllerUtil () {}
    
    /** Deadband then a power curve, so small deflections give fine adjustment and full deflection gives full rate. */
    static double shapeStick(double raw) {
        double input = MathUtil.applyDeadband(raw, OperatorConstants.STICK_DEADBAND);
        return Math.copySign(Math.pow(Math.abs(input), OperatorConstants.STICK_CURVE_EXPONENT), input);
    }

    public static double turretJoystickUpdate(double turretAxisValue){
        return shapeStick(turretAxisValue)
            * OperatorConstants.TURRET_JOYSTICK_RATE_ROT_PER_SEC * RobotConstants.LOOP_PERIOD_SECONDS;
    }

    public static double hoodJoystickUpdate(double hoodAxisValue){
        return shapeStick(hoodAxisValue)
            * OperatorConstants.HOOD_JOYSTICK_RATE_ROT_PER_SEC * RobotConstants.LOOP_PERIOD_SECONDS;
    }

    /** Robot yaw change since last loop, in turret rotations, so the turret holds a field heading. */
    public static double gyroCorrection(double lastYawDeg, double yawDeg) {
        double deltaDeg = yawDeg - lastYawDeg;
        double gyroCorrectionRot = deltaDeg / 360.0 * OperatorConstants.TURRET_GYRO_CORRECTION_GAIN;
        return gyroCorrectionRot;
    }

    /** Bumpers fully past our alliance zone line, so the robot cannot score into the hub. */
    public static boolean beyondAllianceZone(double robotX, Alliance alliance, boolean wasPassing) {
        double distancePastLine = alliance == Alliance.Blue
                ? robotX - FieldConstants.BLUE_ZONE_LINE_X
                : FieldConstants.RED_ZONE_LINE_X - robotX;
        double threshold = RobotConstants.BUMPER_HALF_LENGTH_METERS
                - (wasPassing ? RobotConstants.PASSING_HYSTERESIS_METERS : 0.0);
        return distancePastLine > threshold;
    }
}
