package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;
import frc.robot.subsystems.Indexer;

public class manualIndexer extends Command {
    Indexer m_indexerSubsystem;


    public manualIndexer(Indexer indexerSubsystem) {
        m_indexerSubsystem = indexerSubsystem;
        addRequirements(m_indexerSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_indexerSubsystem.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL);

    }

    @Override
    public void end(boolean interrupted) {
        m_indexerSubsystem.stopIndexer(IndexerType.VERTICAL);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
