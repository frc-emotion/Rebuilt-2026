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
    private final DoubleSupplier shooterRPSOffsetSupplier;
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
        this(indexer, hood, shooter, distanceSupplier, calculator, isAimed, () -> 0.0);
    }

    /**
     * Auto-aim shoot with SOTM: hood + shooter from interpolation tables,
     * shooter RPS adjusted by SOTM offset, indexers gated on isAimed.
     */
    public ShootCommand(Indexer indexer, Hood hood, Shooter shooter,
                        DoubleSupplier distanceSupplier,
                        TurretAimingCalculator calculator,
                        BooleanSupplier isAimed,
                        DoubleSupplier shooterRPSOffsetSupplier) {
        this.indexer = indexer;
        this.hood = hood;
        this.shooter = shooter;
        this.distanceSupplier = distanceSupplier;
        this.calculator = calculator;
        this.isAimed = isAimed;
        this.shooterRPSOffsetSupplier = shooterRPSOffsetSupplier;
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
        this.shooterRPSOffsetSupplier = () -> 0.0;
        this.manualShooterRPS = shooterRPS;
        this.useInterpTables = false;
        addRequirements(indexer, hood, shooter);
    }

    @Override
    public void execute() {
        if (useInterpTables) {
            double dist = distanceSupplier.getAsDouble();
            hood.setHoodAngle(Rotations.of(calculator.getHoodAngleRot(dist)));
            double baseRPS = calculator.getFlywheelRPS(dist);
            double adjustedRPS = baseRPS + shooterRPSOffsetSupplier.getAsDouble();
            shooter.setShooterSpeed(RotationsPerSecond.of(adjustedRPS));
        } else {
            shooter.setShooterSpeed(RotationsPerSecond.of(manualShooterRPS));
            hood.setHoodAngle(Rotations.of(0.0)); // hood flat in manual mode
        }

        if (isAimed.getAsBoolean() && shooter.atShooterSetpoint()) {
            indexer.setIndexerSpeed(-IndexerConstants.HORIZONTAL_INDEXER_SPEED, IndexerType.HORIZONTAL);
            indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL);
            indexer.setIndexerSpeed(IndexerConstants.UPWARD_INDEXER_SPEED, IndexerType.UPWARD);
        } else {
            indexer.setIndexerSpeed(0, IndexerType.HORIZONTAL);
            indexer.setIndexerSpeed(0, IndexerType.VERTICAL);
            indexer.setIndexerSpeed(0, IndexerType.UPWARD);
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
