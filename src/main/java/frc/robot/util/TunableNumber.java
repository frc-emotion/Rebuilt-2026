package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotConstants;

/**
 * A number that is live-adjustable from the dashboard while {@link RobotConstants#kTuningMode} is
 * true and reads as a plain constant in competition mode. Once a value settles it is promoted to
 * the owning subsystem's constants file.
 */
public final class TunableNumber {
  private final double defaultValue;
  private final DoubleEntry entry;

  public TunableNumber(String name, double defaultValue) {
    this.defaultValue = defaultValue;
    if (RobotConstants.kTuningMode) {
      this.entry =
          NetworkTableInstance.getDefault().getTable("Tuning").getDoubleTopic(name).getEntry(defaultValue);
      this.entry.set(defaultValue);
    } else {
      this.entry = null;
    }
  }

  public double get() {
    if (entry == null) {
      return defaultValue;
    }
    return entry.get(defaultValue);
  }
}
