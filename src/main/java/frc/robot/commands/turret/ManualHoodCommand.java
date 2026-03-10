package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class ManualHoodCommand extends Command {
    private final Hood m_hood;
    private final DoubleSupplier m_joystickInput;

    public ManualHoodCommand(Hood hood, DoubleSupplier joystickInput) {
        m_hood = hood;
        this.m_joystickInput = joystickInput;

        addRequirements(m_hood);
    }

    @Override
    public void execute() {
        double input = MathUtil.applyDeadband(m_joystickInput.getAsDouble(), 0.01);

        double clampedInput = MathUtil.clamp(input, -0.7, 0.7);

        m_hood.setHoodManualVoltage(clampedInput);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_hood.stop();
    }
}
