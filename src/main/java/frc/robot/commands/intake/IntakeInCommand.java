package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeInCommand extends Command{
    Intake m_intakeSubsystem;
    public IntakeInCommand(
        Intake intakeSubsystem
    ){
        m_intakeSubsystem = intakeSubsystem;

        addRequirements(m_intakeSubsystem);

    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_intakeSubsystem.stopRoller();
        m_intakeSubsystem.setIntakeSpeed(-0.5);
        
    }

    @Override
    public boolean isFinished(){
        if (m_intakeSubsystem.getCurrentSpike()){
            m_intakeSubsystem.stopIntake();
            return true;
        }
       return false;
    
    }

}
