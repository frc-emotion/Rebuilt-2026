package frc.robot.legacy.commands.indexer;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.legacy.Constants.IndexerConstants;
import frc.robot.legacy.Constants.IndexerConstants.IndexerType;
import frc.robot.legacy.subsystems.Indexer;

public class indexerDefault extends Command {

    Indexer m_indexer;

    BooleanSupplier intakeOut;

    public indexerDefault(Indexer indexerSubsystem, BooleanSupplier intakeOut) {

        this.m_indexer = indexerSubsystem;

        this.intakeOut = intakeOut;

    }

    @Override
    public void execute() {
        if (intakeOut.getAsBoolean()) {
            m_indexer.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED * 0.75, IndexerType.VERTICAL);
        }

        else {
            m_indexer.stopIndexer(IndexerType.VERTICAL);
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
