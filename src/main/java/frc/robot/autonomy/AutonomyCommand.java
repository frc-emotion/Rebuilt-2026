package frc.robot.autonomy;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Wraps the {@link MatchTree} as a WPILib command so it can be scheduled (e.g. as the autonomous
 * routine). It requires no subsystem — the DriveTo nodes schedule their own drivetrain-requiring
 * pathfinding commands, and the skills run through the always-on runtime — so running this never
 * fights the rest of the robot; it just ticks the tree each loop and yields cleanly when it ends
 * (which is how the human-takeover gate stays honored). It plays the whole match (never finishes).
 */
public final class AutonomyCommand extends Command {
  private final MatchTree tree;

  public AutonomyCommand(MatchTree tree) {
    this.tree = tree;
  }

  @Override
  public void initialize() {
    tree.reset();
  }

  @Override
  public void execute() {
    tree.tick();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    tree.onEnd();
  }
}
