package frc.robot.legacy.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.legacy.Constants.IntakeConstants;
import frc.robot.legacy.subsystems.Intake;

public class IntakeInCommand extends Command{
    Intake m_intakeSubsystem;

    public IntakeInCommand(Intake intakeSubsystem){
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize(){
        m_intakeSubsystem.stopRoller();
        m_intakeSubsystem.setIntakeAngle(IntakeConstants.INTAKE_IN_ANGLE);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
