package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants;

@Logged
public class Robot extends TimedRobot {
    private final RobotContainer robotContainer;
    private Command autonomousCommand;

    public Robot() {
        Epilogue.configure(config -> {
            config.root = "Robot";
            config.minimumImportance = RobotConstants.NT_MIN_IMPORTANCE;
        });
        DataLogManager.start();
        SignalLogger.start();
        Epilogue.bind(this);
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.stateMachine.periodic();
    }

    @Override
    public void autonomousInit() {
        robotContainer.stateMachine.onEnable();
        autonomousCommand = robotContainer.getAutonomousCommand();
        CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    @Override
    public void teleopInit() {
        robotContainer.stateMachine.onEnable();
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        robotContainer.vision.updateSimulation(robotContainer.drivetrain.getState().Pose);
    }
}
