package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.RobotConstants;

public class ControllerUtil {
    
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
}
