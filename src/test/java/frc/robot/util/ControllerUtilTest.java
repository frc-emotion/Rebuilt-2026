package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

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
}
