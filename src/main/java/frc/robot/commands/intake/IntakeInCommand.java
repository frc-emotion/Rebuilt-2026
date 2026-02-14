package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
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
        m_intakeSubsystem.setRollerSpeed(0);
        m_intakeSubsystem.setIntakeAngle(IntakeConstants.INTAKE_IN_ANGLE);
        
    }

    @Override
    public boolean isFinished(){
        return m_intakeSubsystem.atSetpoint();
    
    }

}
