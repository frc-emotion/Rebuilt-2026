// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Main robot class with automated Epilogue telemetry.
 * 
 * <p>
 * The @Logged annotation enables automatic logging of all fields in this class
 * and any classes they reference (like RobotContainer and its subsystems).
 * 
 * <p>
 * Telemetry is logged to:
 * <ul>
 * <li>NetworkTables - Live viewing in AdvantageScope/Shuffleboard</li>
 * <li>WPILOG files - Post-match analysis via DataLogManager</li>
 * <li>HOOT files - Phoenix 6 device data via SignalLogger</li>
 * </ul>
 */
@Logged
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    @Logged
    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        // Configure Epilogue with default NetworkTables backend for live viewing
        // NOTE: HootEpilogueBackend only writes to HOOT files, NOT NetworkTables!
        // Using default backend allows live data viewing in AdvantageScope
        Epilogue.configure(config -> {
            // Default backend is NetworkTables - no need to set config.backend
            config.root = "Robot";
        });

        // Start DataLogManager to capture NetworkTables data to disk (.wpilog files)
        // This provides post-match analysis capability for ALL telemetry
        DataLogManager.start();

        // Start Phoenix 6 SignalLogger for high-fidelity CTRE device logging (.hoot
        // files)
        // This captures ALL Phoenix 6 status signals at full CAN rate with timestamps
        SignalLogger.start();

        // Bind Epilogue to robot loop - runs at 50Hz, phase-offset from main loop
        Epilogue.bind(this);

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        // Update drivetrain pose estimation with vision measurements
        m_robotContainer.updateVisionPoseEstimates();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
