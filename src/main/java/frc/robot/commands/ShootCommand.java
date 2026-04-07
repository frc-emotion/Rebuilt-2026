package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.util.TurretAimingCalculator;

/**
 * Spins up shooter + sets hood + feeds indexers.
 *
 * Vision mode (interp tables):
 *   - Aimed: hood + shooter from distance tables, all 3 indexers fire.
 *   - Not aimed: hood stays where operator left it, shooter at FALLBACK_SHOOTER_RPS, all 3 indexers fire.
 * Manual mode: fixed shooter RPS, hood untouched, all indexers always fire.
 */
public class ShootCommand extends Command {

    private static final double FALLBACK_SHOOTER_RPS = 50.0;
    private final CommandSwerveDrivetrain drivetrain;
    private final Indexer indexer;
    private final Hood hood;
    private final Shooter shooter;
    public final BooleanSupplier isAimed;
    private final DoubleSupplier distanceSupplier;
    private final TurretAimingCalculator calculator;
    private final double manualShooterRPS;
    private final boolean useInterpTables;
    private final Turret turret;

    /** Vision mode: hood + shooter from interp tables, indexers gated on isAimed. */
    public ShootCommand(Indexer indexer, Hood hood, Shooter shooter,
                        DoubleSupplier distanceSupplier,
                        TurretAimingCalculator calculator,
                        BooleanSupplier isAimed, 
                        CommandSwerveDrivetrain drivetrain,
                        Turret turret) {
        this.indexer = indexer;
        this.hood = hood;
        this.shooter = shooter;
        this.distanceSupplier = distanceSupplier;
        this.calculator = calculator;
        this.isAimed = isAimed;
        this.manualShooterRPS = 0;
        this.useInterpTables = true;
        this.drivetrain = drivetrain;
        this.turret = turret;
        addRequirements(indexer, hood, shooter);
    }

    /** Manual mode: fixed shooter speed, hood flat, indexers always fire. */
    public ShootCommand(Indexer indexer, Hood hood, Shooter shooter, Turret turret, double shooterRPS, CommandSwerveDrivetrain drivetrain) {
        this.indexer = indexer;
        this.hood = hood;
        this.shooter = shooter;
        this.distanceSupplier = () -> 0;
        this.calculator = null;
        this.isAimed = () -> true;
        this.manualShooterRPS = shooterRPS;
        this.useInterpTables = false;
        this.drivetrain = drivetrain;
        this.turret = turret;
        addRequirements(indexer, hood, shooter);
    }

    @Override
    public void execute() {
        boolean aimed = isAimed.getAsBoolean();
        ChassisSpeeds speeds = drivetrain.getState().Speeds;

        double turretAngleRad = turret.getTurretPosition().getRadians();
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        
        double vClosing = vx * Math.cos(turretAngleRad) + vy * Math.sin(turretAngleRad);

        double dist = distanceSupplier.getAsDouble();

        double effectiveDist = dist - vClosing * TurretConstants.shootingWhileMovingMultiplier; 


       
       
        if (useInterpTables && aimed) {
            
            hood.setHoodAngle(Rotations.of(calculator.getHoodAngleRot(dist)));
            shooter.setShooterSpeed(RotationsPerSecond.of(calculator.getFlywheelRPS(dist)));
        // } else if (useInterpTables) {
        //     // Not aimed: keep hood where operator left it, spin shooter at fallback RPS
        //     shooter.setShooterSpeed(RotationsPerSecond.of(FALLBACK_SHOOTER_RPS));
        } else {
            shooter.setShooterSpeed(RotationsPerSecond.of(manualShooterRPS));
        }

        indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL);

        if (shooter.atShooterSetpoint()) {
            indexer.setIndexerSpeed(IndexerConstants.HORIZONTAL_INDEXER_SPEED, IndexerType.HORIZONTAL);
            indexer.setIndexerSpeed(IndexerConstants.UPWARD_INDEXER_SPEED, IndexerType.UPWARD);
        } else {
            indexer.stopIndexer(IndexerType.HORIZONTAL);
            indexer.stopIndexer(IndexerType.UPWARD);
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
