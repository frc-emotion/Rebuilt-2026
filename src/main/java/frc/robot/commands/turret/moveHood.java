package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class moveHood extends Command {
    Hood m_hoodSubsystem;
    Angle setpoint;

    public moveHood(Hood hoodSubsystem, Angle setpoint) {
        this.m_hoodSubsystem = hoodSubsystem;
        this.setpoint = setpoint;
        addRequirements(m_hoodSubsystem);
    }

    @Override
    public void initialize() {
        setpoint = Degrees.of(0);
    }

    @Override
    public void execute() {
        m_hoodSubsystem.setHoodAngle(setpoint);
    }

    @Override
    public boolean isFinished() {
        return m_hoodSubsystem.atHoodSetpoint();
    }
}
