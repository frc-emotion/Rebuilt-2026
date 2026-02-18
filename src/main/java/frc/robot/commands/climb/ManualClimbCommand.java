package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ManualClimbCommand extends Command {
    
    Climb m_climbSubsystem;
    double voltage;

    public ManualClimbCommand(Climb climbsubsystem, double voltage){
        m_climbSubsystem = climbsubsystem;
        this.voltage = voltage;
        addRequirements(m_climbSubsystem);
    }

    @Override
    public void execute(){
        m_climbSubsystem.setManualVoltage(voltage);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}