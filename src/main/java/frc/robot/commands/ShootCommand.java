package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;
import frc.robot.subsystems.Indexer;

/**
 * Feeds all three indexer motors to shoot — only when the aim gate returns true.
 * Requires only the indexer subsystem so the auto-aim command keeps running uninterrupted.
 */
public class ShootCommand extends Command {

    private final Indexer indexer;
    private final BooleanSupplier isAimed;

    public ShootCommand(Indexer indexer, TurretAutoAimCommand autoAim) {
        this(indexer, autoAim::isAimed);
    }

    public ShootCommand(Indexer indexer, OdometryAutoAimCommand autoAim) {
        this(indexer, autoAim::isAimed);
    }

    public ShootCommand(Indexer indexer, BooleanSupplier isAimedSupplier) {
        this.indexer = indexer;
        this.isAimed = isAimedSupplier;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        if (isAimed.getAsBoolean()) {
            indexer.setIndexerSpeed(IndexerConstants.HORIZONTAL_INDEXER_SPEED, IndexerType.HORIZONTAL);
            // indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL); // DISABLED
            indexer.setIndexerSpeed(IndexerConstants.UPWARD_INDEXER_SPEED, IndexerType.UPWARD);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
