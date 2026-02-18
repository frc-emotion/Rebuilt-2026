package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret; 

public class rotateTurret extends Command{
    Turret m_turretSubsystem; 
    Angle setpoint; 

    public rotateTurret(Turret turretSubsystem , Angle setpoint){
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
        m_turretSubsystem.moveTurret(setpoint);
    }

    @Override
    public boolean isFinished(){
        return m_turretSubsystem.atTurretSetpoint();
    }
}
