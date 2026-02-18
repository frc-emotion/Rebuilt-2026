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
        // Get the value from the joystick supplier
        double input = m_joystickInput.getAsDouble();

        // Clamp power to 30% for safety
        double clampedInput = MathUtil.clamp(input, -0.3, 0.3);

        // Send it to the subsystem
        m_hood.setHoodManualVoltage(clampedInput);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
