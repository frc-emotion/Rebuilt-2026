package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret; 

public class moveHood extends Command{
    Turret m_turretSubsystem; 
    Angle setpoint; 

    public moveHood(Turret turretSubsystem , Angle setpoint){
        this.m_turretSubsystem = turretSubsystem;
        this.setpoint = setpoint;
        addRequirements(m_turretSubsystem);
    }

    @Override
    public void initialize(){
        setpoint = Degrees.of(0);
    }

    @Override
    public void execute(){
        m_turretSubsystem.setHoodAngle(setpoint);
    }

    @Override
    public boolean isFinished(){
        return m_turretSubsystem.atHoodSetpoint();
    }
}
