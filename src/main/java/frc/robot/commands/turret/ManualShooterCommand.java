package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret;

public class ManualShooterCommand extends Command {

    Turret m_turretSubsystem;
    DoubleSupplier trigger;

    public ManualShooterCommand(Turret turretSubsystem, DoubleSupplier trigger) {
        this.m_turretSubsystem = turretSubsystem;
        this.trigger = trigger;
        addRequirements(m_turretSubsystem);
    }

    @Override
    public void execute() {
        double speed = trigger.getAsDouble() * TurretConstants.MAX_SHOOTER_RPS;
        m_turretSubsystem.setShooterSpeed(RotationsPerSecond.of(speed));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}