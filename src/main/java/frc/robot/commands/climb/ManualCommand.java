package frc.robot.commands.climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ManualCommand extends Command {
    
    Climb m_climbSubsystem;

    DoubleSupplier y;


    public ManualCommand(DoubleSupplier operator, Climb climb){
        m_climbSubsystem = climb;

        this.y = operator;

        addRequirements(m_climbSubsystem);
        
    }

    @Override
    public void execute(){
        m_climbSubsystem.setManualVoltage(y.getAsDouble());

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
