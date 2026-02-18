package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class runFlyWheel extends Command {
    Shooter m_shooterSubsystem;
    double speed;

    public runFlyWheel(Shooter shooterSubsystem, double speed) {
        this.m_shooterSubsystem = shooterSubsystem;
        this.speed = speed;
        addRequirements(m_shooterSubsystem);
    }


    @Override
    public void execute() {
        m_shooterSubsystem.setShooterSpeed(RotationsPerSecond.of(speed));
    }

    @Override
    public boolean isFinished() {
        return m_shooterSubsystem.atShooterSetpoint();
    }
}
