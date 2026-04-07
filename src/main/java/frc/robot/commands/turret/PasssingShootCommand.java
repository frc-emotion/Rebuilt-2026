package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

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
    private final BooleanSupplier isPassingLocked;

    public PasssingShootCommand(Shooter shooterSubsystem, Hood hoodSubsystem,
                                Indexer indexerSubsystem, BooleanSupplier isPassingLocked) {
        m_shooterSubsystem = shooterSubsystem;
        m_hoodSubsystem = hoodSubsystem;
        m_indexerSubsystem = indexerSubsystem;
        this.isPassingLocked = isPassingLocked;

        addRequirements(shooterSubsystem, hoodSubsystem, indexerSubsystem);
    }


    public void execute(){
        // Flywheel + hood spin up immediately so they're ready when turret locks
        m_shooterSubsystem.setShooterSpeed(RotationsPerSecond.of(100));
        m_hoodSubsystem.setHoodAngle(Rotations.of(0.075));

        // Only feed ball once turret is locked onto the alliance side
        if (isPassingLocked.getAsBoolean()) {
            m_indexerSubsystem.setIndexerSpeed(IndexerConstants.VERTICAL_INDEXER_SPEED, IndexerType.VERTICAL);
            if (m_shooterSubsystem.atShooterSetpoint()) {
                m_indexerSubsystem.setIndexerSpeed(IndexerConstants.HORIZONTAL_INDEXER_SPEED, IndexerType.HORIZONTAL);
                m_indexerSubsystem.setIndexerSpeed(IndexerConstants.UPWARD_INDEXER_SPEED, IndexerType.UPWARD);
            }
        }
    }

    public boolean isFinished(){
        return false; 
    }

    public void end(){
        m_shooterSubsystem.stop();
        m_indexerSubsystem.stop();
        m_hoodSubsystem.setHoodAngle(Rotations.of(0.0));
    }
    
}
