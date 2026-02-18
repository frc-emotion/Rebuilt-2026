package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret; 


public class ManualHoodCommand extends Command {
    private final Turret m_turret;
    private final DoubleSupplier m_joystickInput;

    public ManualHoodCommand(Turret turret, DoubleSupplier joystickInput) {
        m_turret = turret;
        this.m_joystickInput = joystickInput;

        addRequirements(m_turret);
    }


    @Override
    public void execute() {
    // Get the value from the joystick supplier
    double input = m_joystickInput.getAsDouble();

    double clampedInput = MathUtil.clamp(input, -0.3, 0.3);
    
    // Send it to the subsystem
    m_turret.setHoodManualVoltage(clampedInput);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
