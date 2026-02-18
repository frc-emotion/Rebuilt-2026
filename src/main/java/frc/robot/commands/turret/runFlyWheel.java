package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class runFlyWheel extends Command{
    Turret m_turretSubsystem; 
    double speed; 

    public runFlyWheel(Turret turretSubsystem , double speed){
        this.m_turretSubsystem = turretSubsystem;
        this.speed = speed;
        addRequirements(m_turretSubsystem);
    }

    @Override
    public void initialize(){
        speed = 0.0;
    }

    @Override
    public void execute(){
        m_turretSubsystem.setShooterSpeed(RotationsPerSecond.of(speed));
    }

    @Override
    public boolean isFinished(){
        return m_turretSubsystem.atShooterSetpoint();
    }
}
