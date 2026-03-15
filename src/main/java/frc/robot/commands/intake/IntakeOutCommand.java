package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeOutCommand extends Command{
    Intake m_intakeSubsystem;

    public IntakeOutCommand(
        Intake intakeSubsystem
    ){
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize(){
        m_intakeSubsystem.setIntakeAngle(IntakeConstants.INTAKE_OUT_ANGLE);
        m_intakeSubsystem.setRollerSpeed(IntakeConstants.INTAKE_ROLLER_VELOCITY);
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished(){
        return m_intakeSubsystem.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_intakeSubsystem.stopRoller();
        }
    }
}
