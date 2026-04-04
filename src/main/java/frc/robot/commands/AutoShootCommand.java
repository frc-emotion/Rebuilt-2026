package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TurretAimingCalculator;

/**
 * Keeps the shooter spun up and hood positioned based on live vision distance.
 *
 * Designed for autonomous — runs continuously so the flywheel and hood are
 * always tracking the interpolation tables. When this command is active the
 * superstructure is "hot": the instant a separate command fires the indexers
 * the ball leaves immediately with no spin-up delay.
 *
 * Does NOT require or touch the Indexer subsystem so indexer commands can be
 * composed independently (e.g. sequential auto routines that feed one ball
 * at a time).
 */
public class AutoShootCommand extends Command {

    private final Shooter shooter;
    private final Hood hood;
    private final DoubleSupplier distanceSupplier;
    private final TurretAimingCalculator calculator;

    /**
     * @param shooter          Shooter subsystem
     * @param hood             Hood subsystem
     * @param distanceSupplier Supplies distance to hub in meters (typically Vision::getDistanceToHub)
     * @param calculator       Interpolation tables for flywheel RPS and hood angle
     */
    public AutoShootCommand(Shooter shooter, Hood hood,
                            DoubleSupplier distanceSupplier,
                            TurretAimingCalculator calculator) {
        this.shooter = shooter;
        this.hood = hood;
        this.distanceSupplier = distanceSupplier;
        this.calculator = calculator;
        addRequirements(shooter, hood);
    }

    @Override
    public void execute() {
        double dist = distanceSupplier.getAsDouble();
        shooter.setShooterSpeed(RotationsPerSecond.of(calculator.getFlywheelRPS(dist)));
        hood.setHoodAngle(Rotations.of(calculator.getHoodAngleRot(dist)));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hood.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /** True when both shooter and hood have reached their interpolated setpoints. */
    public boolean isReady() {
        return shooter.atShooterSetpoint() && hood.atHoodSetpoint();
    }
}
