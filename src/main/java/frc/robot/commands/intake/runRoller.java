package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class runRoller extends Command {
    Intake m_intakeSubsystem; 


    public runRoller(Intake intake){
        m_intakeSubsystem = intake; 
    }

    public void execute(){
        m_intakeSubsystem.setRollerSpeed((IntakeConstants.INTAKE_ROLLER_VELOCITY));
    }

    public void intialize(){

    }

    public boolean isFinished(){
        return false;
    }
    
}
