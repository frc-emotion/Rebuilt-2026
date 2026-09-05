package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.StateMachine.IndexerState;
import frc.robot.StateMachine.IntakeState;
import frc.robot.StateMachine.Resolution;
import frc.robot.StateMachine.ShooterState;

class StateMachineTest {
    private static Resolution resolve(boolean intake, boolean shoot, boolean clear, boolean intakeOut, boolean atSpeed) {
        return StateMachine.resolve(intake, shoot, clear, intakeOut, atSpeed);
    }

    @Test
    void idle() {
        assertEquals(new Resolution(IntakeState.STOWED, IndexerState.STOPPED, ShooterState.IDLE),
                resolve(false, false, false, false, false));
    }

    @Test
    void intakeDeployingDoesNotFeedUntilOut() {
        assertEquals(new Resolution(IntakeState.DEPLOYING, IndexerState.STOPPED, ShooterState.IDLE),
                resolve(true, false, false, false, false));
        assertEquals(new Resolution(IntakeState.DEPLOYED, IndexerState.FEED_VERTICAL, ShooterState.IDLE),
                resolve(true, false, false, true, false));
    }

    @Test
    void shootSpinsUpWithVerticalOnlyThenFeedsAll() {
        assertEquals(new Resolution(IntakeState.STOWED, IndexerState.FEED_VERTICAL, ShooterState.SPINNING_UP),
                resolve(false, true, false, false, false));
        assertEquals(new Resolution(IntakeState.STOWED, IndexerState.FEED_ALL, ShooterState.AT_SPEED),
                resolve(false, true, false, false, true));
    }

    @Test
    void clearOverridesIndexersOnly() {
        assertEquals(new Resolution(IntakeState.DEPLOYED, IndexerState.CLEARING, ShooterState.AT_SPEED),
                resolve(true, true, true, true, true));
        assertEquals(new Resolution(IntakeState.STOWED, IndexerState.CLEARING, ShooterState.IDLE),
                resolve(false, false, true, false, false));
    }

    @Test
    void stickCurveIsDeadbandedAndMonotonic() {
        assertEquals(0.0, StateMachine.shapeStick(0.05));
        assertEquals(1.0, StateMachine.shapeStick(1.0), 1e-9);
        assertEquals(-1.0, StateMachine.shapeStick(-1.0), 1e-9);
        double small = StateMachine.shapeStick(0.3);
        double large = StateMachine.shapeStick(0.6);
        assertTrue(small > 0 && large > small);
    }
}
