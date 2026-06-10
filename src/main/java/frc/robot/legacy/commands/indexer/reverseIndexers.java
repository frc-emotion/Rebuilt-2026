package frc.robot.legacy.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.legacy.Constants.IndexerConstants;
import frc.robot.legacy.Constants.IndexerConstants.IndexerType;
import frc.robot.legacy.Constants.TurretConstants;
import frc.robot.legacy.subsystems.Indexer;
import frc.robot.legacy.subsystems.Shooter;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class reverseIndexers extends Command {
    Indexer m_indexerSubsystem;
    Shooter m_shooterSubsystem;

    public reverseIndexers(Indexer indexerSubsystem, Shooter shooterSubsystem) {
        m_indexerSubsystem = indexerSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_indexerSubsystem, m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_indexerSubsystem.setIndexerSpeed(-IndexerConstants.HORIZONTAL_INDEXER_SPEED * 0.5, IndexerType.HORIZONTAL);
        m_indexerSubsystem.setIndexerSpeed(-IndexerConstants.VERTICAL_INDEXER_SPEED * 0.5, IndexerType.VERTICAL);
        m_indexerSubsystem.setIndexerSpeed(-IndexerConstants.UPWARD_INDEXER_SPEED * 0.5, IndexerType.UPWARD);
        m_shooterSubsystem.setShooterSpeed(RotationsPerSecond.of(TurretConstants.MAX_SHOOTER_RPS));
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stop();
        m_indexerSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
