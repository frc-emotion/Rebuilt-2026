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
// import com.bionanomics.refinery.mcp.RoboRioMcpServer;

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

    /**
     * MATCH_MODE controls telemetry depth on NetworkTables.
     * - true  = CRITICAL only on NT (minimal bandwidth — competition matches)
     * - false = DEBUG + CRITICAL on NT (full per-motor diagnostics — pit/practice)
     *
     * ALL data is ALWAYS written to WPILOG + HOOT files regardless of this flag.
     */
    private static final boolean MATCH_MODE = false;

    private final RobotContainer m_robotContainer;

    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        Epilogue.configure(config -> {
            config.root = "Robot";
            config.minimumImportance = MATCH_MODE
                    ? Logged.Importance.CRITICAL
                    : Logged.Importance.DEBUG;
        });

        DataLogManager.start();
        SignalLogger.start();
        Epilogue.bind(this);

        m_robotContainer = new RobotContainer();

        System.out.println("[TELEMETRY] MATCH_MODE=" + MATCH_MODE
                + " -> NT publishes " + (MATCH_MODE ? "CRITICAL only" : "ALL (DEBUG+CRITICAL)"));

        // RoboRioMcpServer.start();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        // Hot-reload any constants changed in Elastic dashboard
        m_robotContainer.tuner.checkForChanges();

        if (m_robotContainer.isVisionPoseEstimationEnabled()) {
            m_robotContainer.updateVisionPoseEstimates();
        }
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
        m_robotContainer.faultMonitor.clearAllStickyFaults();
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
