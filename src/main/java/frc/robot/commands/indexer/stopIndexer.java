package frc.robot.commands.indexer;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;
import frc.robot.subsystems.Indexer;

public class stopIndexer extends Command {
    Indexer m_indexerSubsystem; 

    public stopIndexer(Indexer indexerSubsystem){
        this.m_indexerSubsystem = indexerSubsystem;
        addRequirements(m_indexerSubsystem);
    }
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        m_indexerSubsystem.setIndexerSpeed(0, IndexerType.VERTICAL);
        m_indexerSubsystem.setIndexerSpeed(0, IndexerType.HORIZONTAL);
        m_indexerSubsystem.setIndexerSpeed(0, IndexerType.UPWARD);
       

    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
