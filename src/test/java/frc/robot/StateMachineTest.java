package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.StateMachine.IndexerState;
import frc.robot.StateMachine.IntakeState;
import frc.robot.StateMachine.Resolution;
import frc.robot.StateMachine.ShooterState;
import frc.robot.subsystems.Vision;

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

    private static final Pose2d ORIGIN_FACING_POSITIVE_X = new Pose2d();

    /** Turret positions a full rotation apart point the same way. */
    private static void assertSameDirection(double expectedRot, double actualRot) {
        assertEquals(0.0, Math.IEEEremainder(actualRot - expectedRot, 1.0), 1e-9);
    }

    private static double turretRotToward(Pose2d robotPose, Translation2d target) {
        return Vision.turretRotForBearing(Vision.bearingRot(robotPose, target));
    }

    @Test
    void hubBehindKeepsTurretAtBootHeading() {
        assertSameDirection(0.0, turretRotToward(ORIGIN_FACING_POSITIVE_X, new Translation2d(-1, 0)));
    }

    @Test
    void hubOnLeftTurnsTurretClockwiseQuarter() {
        assertSameDirection(0.25, turretRotToward(ORIGIN_FACING_POSITIVE_X, new Translation2d(0, 1)));
    }

    @Test
    void hubOnRightTurnsTurretCounterclockwiseQuarter() {
        assertSameDirection(-0.25, turretRotToward(ORIGIN_FACING_POSITIVE_X, new Translation2d(0, -1)));
    }

    @Test
    void hubAheadTurnsTurretHalfway() {
        assertSameDirection(0.5, turretRotToward(ORIGIN_FACING_POSITIVE_X, new Translation2d(1, 0)));
    }

    @Test
    void robotHeadingIsRemovedFromBearing() {
        Pose2d robotFacingPositiveY = new Pose2d(2.0, 2.0, Rotation2d.fromRotations(0.25));
        Translation2d hub = new Translation2d(4.647, 4.035);
        double expectedBearingRot = Math.atan2(2.035, 2.647) / (2 * Math.PI) - 0.25;
        assertEquals(expectedBearingRot, Vision.bearingRot(robotFacingPositiveY, hub), 1e-9);
        assertSameDirection(0.5 - expectedBearingRot, turretRotToward(robotFacingPositiveY, hub));
    }

    @Test
    void turretAtBootHeadingPointsOutTheBack() {
        Pose2d robotFacingPositiveY = new Pose2d(1.0, 1.0, Rotation2d.fromRotations(0.25));
        assertSameDirection(0.75, Vision.turretFieldPose(robotFacingPositiveY, 0.0).getRotation().getRotations());
    }

    @Test
    void aimedTurretPoseFacesTheHub() {
        Pose2d robotPose = new Pose2d(2.0, 2.0, Rotation2d.fromRotations(0.1));
        Translation2d hub = new Translation2d(4.647, 4.035);
        double aimedTurretRot = turretRotToward(robotPose, hub);
        Rotation2d fieldAngleToHub = hub.minus(robotPose.getTranslation()).getAngle();
        assertSameDirection(fieldAngleToHub.getRotations(),
                Vision.turretFieldPose(robotPose, aimedTurretRot).getRotation().getRotations());
    }
}
