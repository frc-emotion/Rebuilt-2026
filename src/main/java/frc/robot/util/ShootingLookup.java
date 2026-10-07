package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.ShotPoint;

public final class ShootingLookup {

    private final InterpolatingDoubleTreeMap flywheelRPSTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodAngleTable = new InterpolatingDoubleTreeMap();

    public ShootingLookup() {
        for (ShotPoint point : RobotConstants.SHOT_TABLE) {
            flywheelRPSTable.put(point.distanceMeters(), point.flywheelRps());
            hoodAngleTable.put(point.distanceMeters(), point.hoodRot());
        }
    }

    public double getHoodAngle(double distanceMeters) {
        return hoodAngleTable.get(distanceMeters);
    }

    public double getShooterSpeed(double distanceMeters) {
        return flywheelRPSTable.get(distanceMeters);
    }
}
