package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class PasssingShootCommand extends Command {
    private Shooter m_shooterSubsystem; 
    private Hood m_hoodSubsystem;
    private Indexer m_indexerSubsystem;

    public PasssingShootCommand(Shooter shooterSubsystem , Hood hoodSubsystem, Indexer indexerSubsystem){
        m_shooterSubsystem = shooterSubsystem;
        m_hoodSubsystem = hoodSubsystem; 
        m_indexerSubsystem = indexerSubsystem;

        addRequirements(shooterSubsystem , hoodSubsystem, indexerSubsystem);
    }


    public void execute(){
        m_shooterSubsystem.setShooterSpeed(RotationsPerSecond.of(100));
        m_hoodSubsystem.setHoodAngle(Rotations.of(0.075));
        m_indexerSubsystem.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL);
        if (m_shooterSubsystem.atShooterSetpoint() && m_hoodSubsystem.atHoodSetpoint()){
            m_indexerSubsystem.setIndexerSpeed(IndexerConstants.HORIZONTAL_INDEXER_SPEED, IndexerType.HORIZONTAL);
            m_indexerSubsystem.setIndexerSpeed(IndexerConstants.UPWARD_INDEXER_SPEED, IndexerType.UPWARD);
        }
    }

    public boolean isFinished(){
        return false; 
    }
    
}
