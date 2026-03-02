package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;

/**
 * A tunable number that publishes to NetworkTables (visible in Elastic) and
 * persists across reboots via Preferences.
 * 
 * <p>Usage:
 * <pre>{@code
 * TunableNumber kP = new TunableNumber("Turret/kP", 0.01);
 * // In periodic:
 * if (kP.hasChanged()) {
 *     applyNewConfig(kP.get());
 * }
 * }</pre>
 */
public class TunableNumber {
    private static final String TABLE_NAME = "Tuning";

    private final String key;
    private final double defaultValue;
    private final NetworkTableEntry entry;
    private double lastValue;

    /**
     * @param key          Hierarchical key, e.g. "Turret/kP" or "Hood/MotionMagic/CruiseVelocity"
     * @param defaultValue Used on first boot (before any Preferences are saved)
     */
    public TunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;

        // Load persisted value (or default on first boot)
        double initial = Preferences.getDouble(key, defaultValue);

        // Publish to NetworkTables under /Tuning/<key> for Elastic display
        entry = NetworkTableInstance.getDefault()
                .getTable(TABLE_NAME)
                .getEntry(key);
        entry.setDouble(initial);
        entry.setDefaultDouble(initial);

        lastValue = initial;
    }

    /** Get the current value (reads from NetworkTables — reflects Elastic edits). */
    public double get() {
        return entry.getDouble(defaultValue);
    }

    /**
     * Returns true if the value changed since last call to hasChanged().
     * Also persists the new value to Preferences so it survives reboot.
     */
    public boolean hasChanged() {
        double current = get();
        if (Math.abs(current - lastValue) > 1e-9) {
            lastValue = current;
            Preferences.setDouble(key, current);
            return true;
        }
        return false;
    }

    /** Force-set the value (from code). Updates both NT and Preferences. */
    public void set(double value) {
        entry.setDouble(value);
        Preferences.setDouble(key, value);
        lastValue = value;
    }

    /** Reset to default value. */
    public void reset() {
        set(defaultValue);
    }

    public String getKey() { return key; }
    public double getDefault() { return defaultValue; }
}
