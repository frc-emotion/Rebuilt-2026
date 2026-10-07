package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;

class ControllerUtilTest {
    @Test
    void stickCurveIsDeadbandedAndMonotonic() {
        assertEquals(0.0, ControllerUtil.shapeStick(0.05));
        assertEquals(1.0, ControllerUtil.shapeStick(1.0), 1e-9);
        assertEquals(-1.0, ControllerUtil.shapeStick(-1.0), 1e-9);
        double small = ControllerUtil.shapeStick(0.3);
        double large = ControllerUtil.shapeStick(0.6);
        assertTrue(small > 0 && large > small);
    }

    @Test
    void passingNeedsBumpersFullyPastTheZoneLine() {
        double blueEdge = FieldConstants.BLUE_ZONE_LINE_X + RobotConstants.BUMPER_HALF_LENGTH_METERS;
        double redEdge = FieldConstants.RED_ZONE_LINE_X - RobotConstants.BUMPER_HALF_LENGTH_METERS;
        assertFalse(ControllerUtil.beyondAllianceZone(blueEdge - 0.01, Alliance.Blue, false));
        assertTrue(ControllerUtil.beyondAllianceZone(blueEdge + 0.01, Alliance.Blue, false));
        assertFalse(ControllerUtil.beyondAllianceZone(redEdge + 0.01, Alliance.Red, false));
        assertTrue(ControllerUtil.beyondAllianceZone(redEdge - 0.01, Alliance.Red, false));
    }

    @Test
    void passingStaysOnUntilBackInsideByHysteresis() {
        double blueEdge = FieldConstants.BLUE_ZONE_LINE_X + RobotConstants.BUMPER_HALF_LENGTH_METERS;
        double hysteresis = RobotConstants.PASSING_HYSTERESIS_METERS;
        assertTrue(ControllerUtil.beyondAllianceZone(blueEdge - hysteresis / 2, Alliance.Blue, true));
        assertFalse(ControllerUtil.beyondAllianceZone(blueEdge - hysteresis - 0.01, Alliance.Blue, true));
    }
}
