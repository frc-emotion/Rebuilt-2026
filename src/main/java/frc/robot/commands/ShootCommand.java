package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.TurretAimingCalculator;

/**
 * Spins up shooter + sets hood from interpolation tables + feeds indexers when aimed.
 * Requires hood, shooter, and indexer so it can command all three without conflicting
 * with the turret auto-aim (which only requires turret).
 *
 * <p>Two construction patterns:
 * <ul>
 *   <li><b>Auto modes</b>: distance supplier + calculator → interp-table hood/shooter</li>
 *   <li><b>Manual mode</b>: fixed shooter RPS, hood holds current position, no aim gate</li>
 * </ul>
 */
public class ShootCommand extends Command {

    private final Indexer indexer;
    private final Hood hood;
    private final Shooter shooter;
    private final BooleanSupplier isAimed;
    private final DoubleSupplier distanceSupplier;
    private final TurretAimingCalculator calculator;
    private final double manualShooterRPS;
    private final boolean useInterpTables;

    /**
     * Auto-aim shoot: hood + shooter from interpolation tables, indexers gated on isAimed.
     */
    public ShootCommand(Indexer indexer, Hood hood, Shooter shooter,
                        DoubleSupplier distanceSupplier,
                        TurretAimingCalculator calculator,
                        BooleanSupplier isAimed) {
        this.indexer = indexer;
        this.hood = hood;
        this.shooter = shooter;
        this.distanceSupplier = distanceSupplier;
        this.calculator = calculator;
        this.isAimed = isAimed;
        this.manualShooterRPS = 0;
        this.useInterpTables = true;
        addRequirements(indexer, hood, shooter);
    }

    /**
     * Manual shoot: fixed shooter speed, hood holds current position, indexers always fire.
     */
    public ShootCommand(Indexer indexer, Hood hood, Shooter shooter, double shooterRPS) {
        this.indexer = indexer;
        this.hood = hood;
        this.shooter = shooter;
        this.distanceSupplier = () -> 0;
        this.calculator = null;
        this.isAimed = () -> true;
        this.manualShooterRPS = shooterRPS;
        this.useInterpTables = false;
        addRequirements(indexer, hood, shooter);
    }

    @Override
    public void execute() {
        if (useInterpTables) {
            double dist = distanceSupplier.getAsDouble();
            hood.setHoodAngle(Rotations.of(calculator.getHoodAngleRot(dist)));
            shooter.setShooterSpeed(RotationsPerSecond.of(calculator.getFlywheelRPS(dist)));
        } else {
            shooter.setShooterSpeed(RotationsPerSecond.of(manualShooterRPS));
        }

        if (isAimed.getAsBoolean()) {
            indexer.setIndexerSpeed(-IndexerConstants.HORIZONTAL_INDEXER_SPEED, IndexerType.HORIZONTAL);
            indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL); // DISABLED
            indexer.setIndexerSpeed(IndexerConstants.UPWARD_INDEXER_SPEED, IndexerType.UPWARD);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
