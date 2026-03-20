package frc.robot.commands.climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

/**
 * Manual voltage-based climb command using joystick input.
 * 
 * <p>Sign convention (motor is CounterClockwise_Positive):
 * <ul>
 *   <li>Positive input → positive voltage → CCW → climb UP</li>
 *   <li>Negative input → negative voltage → CW → climb DOWN</li>
 * </ul>
 * 
 * <p>Includes deadband filtering and gravity compensation.
 * This replaces the position-based PID climb for simple manual control.
 */
public class VoltageClimbCommand extends Command {

    private final Climb m_climb;
    private final DoubleSupplier m_inputSupplier;

    /**
     * @param climb         The climb subsystem
     * @param inputSupplier Joystick axis supplier (-1 to +1). Positive = up, negative = down.
     */
    public VoltageClimbCommand(Climb climb, DoubleSupplier inputSupplier) {
        m_climb = climb;
        m_inputSupplier = inputSupplier;
        addRequirements(m_climb);
    }

    @Override
    public void initialize() {
        System.out.println("[CLIMB] VoltageClimbCommand started (default command running)");
    }

    @Override
    public void execute() {
        double input = m_inputSupplier.getAsDouble();

        // Apply deadband
        if (Math.abs(input) < ClimbConstants.MANUAL_CLIMB_DEADBAND) {
            input = 0.0;
        }

        // Scale joystick input to voltage and add gravity compensation
        double volts = (input * ClimbConstants.MANUAL_CLIMB_VOLTAGE) + ClimbConstants.CLIMB_GRAVITY_COMP_VOLTS;

        m_climb.setClimbVoltage(volts);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_climb.stop();
    }
}
