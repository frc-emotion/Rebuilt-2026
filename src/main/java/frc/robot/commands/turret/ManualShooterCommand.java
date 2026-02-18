package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Shooter;

public class ManualShooterCommand extends Command {

    Shooter m_shooterSubsystem;
    DoubleSupplier trigger;

    public ManualShooterCommand(Shooter shooterSubsystem, DoubleSupplier trigger) {
        this.m_shooterSubsystem = shooterSubsystem;
        this.trigger = trigger;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void execute() {
        double speed = trigger.getAsDouble() * TurretConstants.MAX_SHOOTER_RPS;
        m_shooterSubsystem.setShooterSpeed(RotationsPerSecond.of(speed));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}