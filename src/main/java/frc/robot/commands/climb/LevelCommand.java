package frc.robot.commands.climb;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.ClimbLevel;
import frc.robot.subsystems.Climb;

public class LevelCommand extends Command{
    Climb m_climbSubsystem;
    private boolean firstPhaseComplete;
    private boolean started;
    private ClimbLevel climbLevel;

    public LevelCommand(
        Climb climbSubsystem,
        ClimbLevel level
    ){
        m_climbSubsystem = climbSubsystem;
        climbLevel = level;

        addRequirements(m_climbSubsystem);

    }


    @Override
    public void initialize(){
        started = false;
        firstPhaseComplete = false;

    }

    @Override
    public void execute(){
        if (!started && !firstPhaseComplete){
            m_climbSubsystem.setClimbPosition(climbLevel.inner);
            started = true;
        }
        else if ((started) && (!firstPhaseComplete)){
            firstPhaseComplete = m_climbSubsystem.atSetpoint();
        }
        else if (firstPhaseComplete && started){
            m_climbSubsystem.setClimbPosition(climbLevel.outer);
            started = false;
        }
            
    }

    @Override
    public boolean isFinished(){
        return ((firstPhaseComplete) && m_climbSubsystem.atSetpoint());
    
    }

}
