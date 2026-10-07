package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants {
    private FieldConstants() {}

    public static final Translation2d BLUE_HUB = new Translation2d(Units.inchesToMeters(182.97), Units.inchesToMeters(158.844));
    public static final Translation2d RED_HUB = new Translation2d(Units.inchesToMeters(469.115), Units.inchesToMeters(158.844));

    public static final double BLUE_ZONE_LINE_X = Units.inchesToMeters(158.6);
    public static final double RED_ZONE_LINE_X = Units.inchesToMeters(651.2 - 158.6);

    /** Passing aims straight down the field toward our own alliance wall. */
    public static final Rotation2d BLUE_PASSING_HEADING = Rotation2d.k180deg;
    public static final Rotation2d RED_PASSING_HEADING = Rotation2d.kZero;
}
