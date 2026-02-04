package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbSequenceCommand extends SequentialCommandGroup {

    public ClimbSequenceCommand(ClimbSubsystem climb) {
        addRequirements(climb);

        // Define tolerances (e.g., within 2 rotations of target)
        double tolerance = 2.0; 

        addCommands(
            // 1. Go UP to MAX
            Commands.run(() -> climb.setPosition(ClimbConstants.MAX_HEIGHT_ROTATIONS), climb)
                .until(() -> climb.atSetpoint(ClimbConstants.MAX_HEIGHT_ROTATIONS, tolerance)),
            
            // Short wait to stabilize (optional)
            new WaitCommand(0.2),

            // 2. Go DOWN all the way
            Commands.run(() -> climb.setPosition(ClimbConstants.BOTTOM_HEIGHT), climb)
                .until(() -> climb.atSetpoint(ClimbConstants.BOTTOM_HEIGHT, tolerance)),

            // 3. Go UP 3/4
            Commands.run(() -> climb.setPosition(ClimbConstants.THREE_QUARTER_HEIGHT), climb)
                .until(() -> climb.atSetpoint(ClimbConstants.THREE_QUARTER_HEIGHT, tolerance)),

            // 4. Go DOWN all the way
            Commands.run(() -> climb.setPosition(ClimbConstants.BOTTOM_HEIGHT), climb)
                .until(() -> climb.atSetpoint(ClimbConstants.BOTTOM_HEIGHT, tolerance)),

            // 5. Go UP 3/4
            Commands.run(() -> climb.setPosition(ClimbConstants.THREE_QUARTER_HEIGHT), climb)
                .until(() -> climb.atSetpoint(ClimbConstants.THREE_QUARTER_HEIGHT, tolerance)),

            // 6. Go DOWN all the way (Final state)
            Commands.run(() -> climb.setPosition(ClimbConstants.BOTTOM_HEIGHT), climb)
                // We don't end immediately here, we keep holding bottom
        );
    }
}