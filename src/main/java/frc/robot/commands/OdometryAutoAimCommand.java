package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.util.TurretAimingCalculator;
import frc.robot.util.TurretAimingCalculator.AimingParameters;

/**
 * Odometry-only auto-aim (NO_VISION mode). Turret tracks hub using only
 * drivetrain pose estimation (wheel odometry). No vision correction.
 * Hood and shooter are commanded from interpolation tables.
 */
@Logged
public class OdometryAutoAimCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final TurretAimingCalculator calculator;

    @Logged private double distanceToHubMeters = 0.0;
    @Logged private double encoderSetpointRot = TurretConstants.TURRET_FORWARD_POSITION;

    public OdometryAutoAimCommand(
            CommandSwerveDrivetrain drivetrain,
            Turret turret,
            Hood hood,
            Shooter shooter) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.calculator = new TurretAimingCalculator();
        addRequirements(turret, hood, shooter);
    }

    @Override
    public void initialize() {
        calculator.clearAllianceCache();
        encoderSetpointRot = turret.getTurretMotor().getPosition().getValueAsDouble();
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        AimingParameters params = calculator.calculate(robotPose);
        distanceToHubMeters = params.distanceToHub();

        encoderSetpointRot = TurretConstants.TURRET_FORWARD_POSITION
                + params.turretAngle().getRotations()
                + TurretConstants.TURRET_AIM_OFFSET;

        turret.moveTurretWithWrap(Rotations.of(encoderSetpointRot), 0.0);

        hood.setHoodAngle(Rotations.of(params.hoodAngle().getRotations()));
        shooter.setShooterSpeed(RotationsPerSecond.of(params.flywheelRPS()));
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        hood.stop();
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean isAimed() {
        double errorRot = Math.abs(
                turret.getTurretMotor().getPosition().getValueAsDouble() - encoderSetpointRot);
        return errorRot < TurretConstants.turretTolerance;
    }

    public double getDistanceToHub() {
        return distanceToHubMeters;
    }
}
