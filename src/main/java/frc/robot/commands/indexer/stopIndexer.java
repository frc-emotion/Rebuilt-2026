package frc.robot.commands.indexer;


import edu.wpi.first.wpilibj2.command.Command;
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

        m_indexerSubsystem.stopIndexer(IndexerType.HORIZONTAL);
        m_indexerSubsystem.stopIndexer(IndexerType.VERTICAL);
        m_indexerSubsystem.stopIndexer(IndexerType.UPWARD);
       

    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
