package frc.robot.commands.turret;



import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
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
        double input = MathUtil.applyDeadband(x.getAsDouble(), 0.01);
        double clampedInput = MathUtil.clamp(input, -1, 1);
        m_turretSubsystem.setTurretVoltage(clampedInput);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        m_turretSubsystem.stop();
    }
}
