package frc.robot.logging;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANID;

/**
 * Centralized fault monitoring for all TalonFX motors.
 * 
 * <p>
 * Monitors registered motors for faults and reports them immediately to:
 * <ul>
 * <li>DriverStation console (warning messages)</li>
 * <li>AdvantageScope via Epilogue telemetry</li>
 * </ul>
 * 
 * <p>
 * Usage:
 * 
 * <pre>
 * // In RobotContainer or subsystem constructor:
 * faultMonitor.register(CANID.FRONT_LEFT_DRIVE, driveMotor);
 * </pre>
 */
@Logged
public class FaultMonitor extends SubsystemBase {

    /** Registered motors: CANID -> TalonFX */
    private final Map<CANID, TalonFX> motors = new HashMap<>();

    /** Cached fault status signals for efficient reading */
    private final Map<CANID, StatusSignal<Integer>> faultSignals = new HashMap<>();

    /** Previously detected faults (to avoid spamming console) */
    private final Map<CANID, Integer> previousFaults = new HashMap<>();

    // ==================
    // EPILOGUE TELEMETRY FIELDS
    // ==================

    /** List of currently faulted motor names */
    @Logged
    private String[] activeFaults = new String[0];

    /** Total number of faults detected this session */
    @Logged
    private int totalFaultCount = 0;

    /** Timestamp of last fault (FPGA time) */
    @Logged
    private double lastFaultTime = 0.0;

    /** Name of the last motor that faulted */
    @Logged
    private String lastFaultMotor = "None";

    /** Description of the last fault */
    @Logged
    private String lastFaultDescription = "None";

    /** Number of motors being monitored */
    @Logged
    private int monitoredMotorCount = 0;

    /**
     * Creates a new FaultMonitor.
     */
    public FaultMonitor() {
        // Subsystem registered automatically
    }

    /**
     * Registers a TalonFX motor for fault monitoring.
     * 
     * @param canId The CANID enum value for this motor
     * @param motor The TalonFX motor instance
     */
    public void register(CANID canId, TalonFX motor) {
        motors.put(canId, motor);
        faultSignals.put(canId, motor.getStickyFaultField());
        previousFaults.put(canId, 0);
        monitoredMotorCount = motors.size();

        // Set update frequency for fault signals (don't need to be super fast)
        motor.getStickyFaultField().setUpdateFrequency(10); // 10 Hz
    }

    /**
     * Registers a TalonFX motor using its CAN ID number.
     * Looks up the CANID enum automatically.
     * 
     * @param canIdNumber The raw CAN ID number
     * @param motor       The TalonFX motor instance
     */
    public void register(int canIdNumber, TalonFX motor) {
        // Find matching CANID enum
        for (CANID canId : CANID.values()) {
            if (canId.getId() == canIdNumber) {
                register(canId, motor);
                return;
            }
        }
        // If no matching enum, log a warning
        DriverStation.reportWarning(
                "[FaultMonitor] Motor with CAN ID " + canIdNumber + " not in CANID registry!",
                false);
    }

    @Override
    public void periodic() {
        List<String> currentFaults = new ArrayList<>();

        for (Map.Entry<CANID, TalonFX> entry : motors.entrySet()) {
            CANID canId = entry.getKey();
            StatusSignal<Integer> faultSignal = faultSignals.get(canId);

            // Refresh and get fault field
            int faultField = faultSignal.refresh().getValue();
            int previousFault = previousFaults.get(canId);

            // Check if there are any faults (non-zero = fault present)
            if (faultField != 0) {
                currentFaults.add(canId.getName());

                // Only log if this is a NEW fault (not previously seen)
                if (faultField != previousFault) {
                    String faultDescription = decodeFaults(faultField);

                    // Log to DriverStation console
                    DriverStation.reportWarning(
                            "⚠️ MOTOR FAULT: " + canId.getName() +
                                    " (ID: " + canId.getId() + ") - " + faultDescription,
                            false);

                    // Update telemetry
                    totalFaultCount++;
                    lastFaultTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                    lastFaultMotor = canId.getName();
                    lastFaultDescription = faultDescription;
                }
            }

            previousFaults.put(canId, faultField);
        }

        // Update active faults list for telemetry
        activeFaults = currentFaults.toArray(new String[0]);
    }

    /**
     * Decodes the fault field bitmask into human-readable fault names.
     */
    private String decodeFaults(int faultField) {
        List<String> faults = new ArrayList<>();

        // TalonFX Sticky Fault bits (from Phoenix 6 documentation)
        if ((faultField & 0x1) != 0)
            faults.add("Hardware");
        if ((faultField & 0x2) != 0)
            faults.add("ProcTemp");
        if ((faultField & 0x4) != 0)
            faults.add("DeviceTemp");
        if ((faultField & 0x8) != 0)
            faults.add("Undervoltage");
        if ((faultField & 0x10) != 0)
            faults.add("BootDuringEnable");
        if ((faultField & 0x20) != 0)
            faults.add("UnlicensedFeature");
        if ((faultField & 0x40) != 0)
            faults.add("BridgeBrownout");
        if ((faultField & 0x80) != 0)
            faults.add("RemoteSensorReset");
        if ((faultField & 0x100) != 0)
            faults.add("MissingDifferentialFX");
        if ((faultField & 0x200) != 0)
            faults.add("RemoteSensorPosOverflow");
        if ((faultField & 0x400) != 0)
            faults.add("OverSupplyV");
        if ((faultField & 0x800) != 0)
            faults.add("UnstableSupplyV");
        if ((faultField & 0x1000) != 0)
            faults.add("ReverseHardLimit");
        if ((faultField & 0x2000) != 0)
            faults.add("ForwardHardLimit");
        if ((faultField & 0x4000) != 0)
            faults.add("ReverseSoftLimit");
        if ((faultField & 0x8000) != 0)
            faults.add("ForwardSoftLimit");
        if ((faultField & 0x10000) != 0)
            faults.add("RemoteSensorDataInvalid");
        if ((faultField & 0x20000) != 0)
            faults.add("FusedSensorOutOfSync");
        if ((faultField & 0x40000) != 0)
            faults.add("StatorCurrLimit");
        if ((faultField & 0x80000) != 0)
            faults.add("SupplyCurrLimit");
        if ((faultField & 0x100000) != 0)
            faults.add("UsingFusedCANcoderWhileUnlicensed");
        if ((faultField & 0x200000) != 0)
            faults.add("StaticBrakeDisabled");

        if (faults.isEmpty()) {
            return "Unknown (0x" + Integer.toHexString(faultField) + ")";
        }

        return String.join(", ", faults);
    }

    /**
     * Clears all sticky faults on all registered motors.
     * Call this after reviewing faults or at the start of a match.
     */
    public void clearAllStickyFaults() {
        for (Map.Entry<CANID, TalonFX> entry : motors.entrySet()) {
            entry.getValue().clearStickyFaults();
            DriverStation.reportWarning(
                    "[FaultMonitor] Cleared sticky faults on " + entry.getKey().getName(),
                    false);
        }
        previousFaults.replaceAll((k, v) -> 0);
        activeFaults = new String[0];
    }

    /**
     * Gets whether any motor currently has a fault.
     */
    public boolean hasActiveFaults() {
        return activeFaults.length > 0;
    }

    /**
     * Gets the total number of faults detected this session.
     */
    public int getTotalFaultCount() {
        return totalFaultCount;
    }
}
