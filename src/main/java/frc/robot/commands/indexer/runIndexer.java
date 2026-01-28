package frc.robot.commands.indexer;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class runIndexer extends Command {
    Indexer m_indexerSubsystem; 

    public runIndexer(Indexer indexerSubsystem){
        this.m_indexerSubsystem = indexerSubsystem;
        addRequirements(m_indexerSubsystem);
    }
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_indexerSubsystem.setHorizontalIndexerSpeed(0.5);
        m_indexerSubsystem.setVerticalIndexerSpeed(0.5);
        m_indexerSubsystem.setUpwardIndexerSpeed(0.5);
       

    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
