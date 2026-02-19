package frc.robot.commands.turret;



import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class ManualTurretCommand extends Command {
    DoubleSupplier x; 
    Turret m_turretSubsystem; 
    public ManualTurretCommand(Turret turretSubsystem, DoubleSupplier x){
        this.x = x;
        this.m_turretSubsystem = turretSubsystem; 
        addRequirements(m_turretSubsystem);
    }

    @Override
    public void execute(){
        m_turretSubsystem.setTurretVoltage(x.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return false;
    } 
    
}
