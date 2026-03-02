package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;
import frc.robot.subsystems.Indexer;

/**
 * Feeds all three indexer motors to shoot — only when turret is aimed and flywheel is at speed.
 * Requires only the indexer subsystem so TurretAutoAimCommand keeps running uninterrupted.
 */
public class ShootCommand extends Command {

    private final Indexer indexer;
    private final TurretAutoAimCommand autoAim;

    public ShootCommand(Indexer indexer, TurretAutoAimCommand autoAim) {
        this.indexer = indexer;
        this.autoAim = autoAim;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        if (autoAim.isAimed()) {
            indexer.setIndexerSpeed(IndexerConstants.HORIZONTAL_INDEXER_SPEED, IndexerType.HORIZONTAL);
            indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL);
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
