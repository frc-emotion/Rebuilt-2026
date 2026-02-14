package frc.robot.commands.climb;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.LevelCommand;
import frc.robot.subsystems.Climb;
import frc.robot.Constants.ClimbConstants.ClimbLevel;
public class Sequence extends SequentialCommandGroup {

    Climb climbSubsystem;
    int level;
    
    public Sequence(
        Climb climbSubsystem,
        int level){
        this.level = level;

        switch (level) {
            case 1 -> 
                    addCommands(
                        new LevelCommand(climbSubsystem, ClimbLevel.LEVEL_1)
                    );
            case 2 ->
                    addCommands(
                        new LevelCommand(climbSubsystem, ClimbLevel.LEVEL_1),
                        new LevelCommand(climbSubsystem, ClimbLevel.LEVEL_2)
                    );
            case 3 ->
                    addCommands(
                        new LevelCommand(climbSubsystem, ClimbLevel.LEVEL_1),
                        new LevelCommand(climbSubsystem, ClimbLevel.LEVEL_2),
                        new LevelCommand(climbSubsystem, ClimbLevel.LEVEL_3)
                    );
        }
    }
}
