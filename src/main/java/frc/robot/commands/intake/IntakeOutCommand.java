package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeOutCommand extends Command{
    Intake m_intakeSubsystem;
    private boolean rollersStarted = false;

    public IntakeOutCommand(Intake intakeSubsystem){
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize(){
        m_intakeSubsystem.setIntakeAngle(IntakeConstants.INTAKE_OUT_ANGLE);
        rollersStarted = false;
    }

    @Override
    public void execute(){
        if (!rollersStarted && m_intakeSubsystem.atSetpoint(IntakeConstants.DEPLOY_TOLERANCE)) {
            m_intakeSubsystem.setRollerSpeed(IntakeConstants.INTAKE_ROLLER_VELOCITY);
            rollersStarted = true;
        }
    }

    @Override
    public void end(boolean interrupted){
        m_intakeSubsystem.stopRoller();
        m_intakeSubsystem.setIntakeAngle(IntakeConstants.INTAKE_IN_ANGLE);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
