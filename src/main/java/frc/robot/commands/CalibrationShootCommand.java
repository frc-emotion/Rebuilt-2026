package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * Calibration command for populating the interpolation tables.
 *
 * <p>Reads hood angle (rotations) and shooter speed (RPS) live from
 * NetworkTables (editable in Elastic dashboard under /Calibration/).
 * Holds turret at position 0, spins up shooter, sets hood, and
 * fires all indexers so you can test shots at each distance.
 *
 * <h3>Workflow</h3>
 * <ol>
 *   <li>Place robot at known distance from hub.</li>
 *   <li>Hold the calibration button (whileTrue binding).</li>
 *   <li>Adjust Hood and ShooterRPS in Elastic until balls score.</li>
 *   <li>Record (distance, hood, shooterRPS) into TurretAimingCalculator.</li>
 *   <li>Move to next distance and repeat.</li>
 * </ol>
 */
public class CalibrationShootCommand extends Command {

    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final Indexer indexer;

    private final DoubleEntry hoodEntry;
    private final DoubleEntry shooterEntry;
    private final DoubleEntry distanceEntry;

    public CalibrationShootCommand(Turret turret, Hood hood, Shooter shooter, Indexer indexer) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(turret, hood, shooter, indexer);

        var table = NetworkTableInstance.getDefault().getTable("Calibration");
        hoodEntry = table.getDoubleTopic("HoodAngleRot").getEntry(0.04);
        shooterEntry = table.getDoubleTopic("ShooterRPS").getEntry(40.0);
        distanceEntry = table.getDoubleTopic("DistanceMeters").getEntry(0.0);

        // Publish defaults so they appear in Elastic immediately
        hoodEntry.setDefault(0.04);
        shooterEntry.setDefault(40.0);
        distanceEntry.setDefault(0.0);
    }

    @Override
    public void execute() {
        // Read live values from Elastic
        double hoodRot = hoodEntry.get();
        double shooterRPS = shooterEntry.get();

        // Hold turret at center (position 0)
        turret.moveTurret(Rotations.of(0.0));

        // Set hood and shooter from Elastic values
        hood.setHoodAngle(Rotations.of(hoodRot));
        shooter.setShooterSpeed(RotationsPerSecond.of(shooterRPS));

        // Fire all indexers
        indexer.setIndexerSpeed(-IndexerConstants.HORIZONTAL_INDEXER_SPEED, IndexerType.HORIZONTAL);
        indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL);
        indexer.setIndexerSpeed(IndexerConstants.UPWARD_INDEXER_SPEED, IndexerType.UPWARD);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        hood.stop();
        shooter.stop();
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
