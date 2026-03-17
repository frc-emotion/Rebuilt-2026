package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import static edu.wpi.first.units.Units.Rotations;

/**
 * Oscillates the intake pivot a small amount back and forth from its stowed
 * position to dislodge balls stuck in the hopper. Runs while held; returns
 * to stowed (INTAKE_IN_ANGLE) on release.
 */
public class IntakeIrrigateCommand extends Command {

    private final Intake intake;
    private final Timer timer = new Timer();

    // Oscillation amplitude in rotations (how far from stowed in each direction)
    private static final double AMPLITUDE_ROT = 0.02;
    // Full cycle period in seconds (back-and-forth)
    private static final double PERIOD_SEC = 0.4;

    private double centerRot;

    public IntakeIrrigateCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        centerRot = IntakeConstants.INTAKE_IN_ANGLE.in(Rotations);
        timer.restart();
    }

    @Override
    public void execute() {
        double t = timer.get();
        // Triangle wave: oscillates between center-AMPLITUDE and center+AMPLITUDE
        double phase = (t % PERIOD_SEC) / PERIOD_SEC;
        double offset = (phase < 0.5)
                ? AMPLITUDE_ROT * (phase * 4 - 1)   // -A to +A
                : AMPLITUDE_ROT * (3 - phase * 4);   // +A to -A
        intake.setIntakeAngle(Rotations.of(centerRot + offset));
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeAngle(IntakeConstants.INTAKE_IN_ANGLE);
        intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
