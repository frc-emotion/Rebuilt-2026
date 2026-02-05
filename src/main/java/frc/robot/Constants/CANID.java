package frc.robot.Constants;

import java.util.HashMap;
import java.util.Map;

/**
 * Centralized enumeration of all CAN device IDs on the robot.
 * 
 * <p>
 * This provides human-readable names for fault reporting and debugging.
 * Use {@link #getName(int)} to lookup a device name by its CAN ID.
 * 
 * <p>
 * <b>WARNING:</b> Some IDs may conflict between subsystems. Verify hardware
 * assignments before deploying to ensure no CAN bus collisions.
 */
public enum CANID {
    // ==================
    // SWERVE DRIVETRAIN
    // ==================
    FRONT_LEFT_DRIVE(26, "Front Left Drive"),
    FRONT_LEFT_STEER(24, "Front Left Steer"),
    FRONT_LEFT_CANCODER(1, "Front Left CANcoder"),

    FRONT_RIGHT_DRIVE(23, "Front Right Drive"),
    FRONT_RIGHT_STEER(15, "Front Right Steer"),
    FRONT_RIGHT_CANCODER(4, "Front Right CANcoder"),

    BACK_LEFT_DRIVE(25, "Back Left Drive"),
    BACK_LEFT_STEER(17, "Back Left Steer"),
    BACK_LEFT_CANCODER(2, "Back Left CANcoder"),

    BACK_RIGHT_DRIVE(18, "Back Right Drive"),
    BACK_RIGHT_STEER(16, "Back Right Steer"),
    BACK_RIGHT_CANCODER(3, "Back Right CANcoder"),

    PIGEON_GYRO(30, "Pigeon Gyro"),

    // ==================
    // INTAKE SUBSYSTEM
    // ==================
    INTAKE_MOTOR(20, "Intake Motor"),
    ROLLER_MOTOR(21, "Roller Motor"),

    // ==================
    // INDEXER SUBSYSTEM
    // Note: IDs 20-22 may conflict with Intake - verify hardware assignments!
    // ==================
    HORIZONTAL_INDEXER(20, "Horizontal Indexer"),
    VERTICAL_INDEXER(21, "Vertical Indexer"),
    UPWARD_INDEXER(22, "Upward Indexer"),

    // ==================
    // TURRET SUBSYSTEM
    // ==================
    SHOOTER_WHEEL(1001, "Shooter Wheel"),
    TURRET_ROTATION(1002, "Turret Rotation"),
    TURRET_ANGLE(1003, "Turret Angle"),

    // ==================
    // CLIMB SUBSYSTEM
    // ==================
    CLIMB_LEADER(31, "Climb Leader"),
    CLIMB_FOLLOWER(32, "Climb Follower");

    private final int id;
    private final String name;

    /** Map for fast lookup of name by CAN ID */
    private static final Map<Integer, String> ID_TO_NAME = new HashMap<>();

    static {
        for (CANID canId : CANID.values()) {
            // Note: If there are duplicate IDs, the last one wins
            ID_TO_NAME.put(canId.getId(), canId.getName());
        }
    }

    CANID(int id, String name) {
        this.id = id;
        this.name = name;
    }

    /**
     * Gets the CAN ID number.
     */
    public int getId() {
        return id;
    }

    /**
     * Gets the human-readable name for this device.
     */
    public String getName() {
        return name;
    }

    /**
     * Looks up a device name by its CAN ID.
     * 
     * @param id The CAN ID to look up
     * @return The device name, or "Unknown Device (ID: X)" if not found
     */
    public static String getName(int id) {
        return ID_TO_NAME.getOrDefault(id, "Unknown Device (ID: " + id + ")");
    }

    /**
     * Checks if a CAN ID is registered.
     * 
     * @param id The CAN ID to check
     * @return true if the ID is in the registry
     */
    public static boolean isRegistered(int id) {
        return ID_TO_NAME.containsKey(id);
    }

    // ==================
    // SWERVE MODULE HELPERS
    // ==================

    /** Swerve module order: FL=0, FR=1, BL=2, BR=3 */
    public static final int[][] SWERVE_IDS = {
            { FRONT_LEFT_DRIVE.id, FRONT_LEFT_STEER.id, FRONT_LEFT_CANCODER.id },
            { FRONT_RIGHT_DRIVE.id, FRONT_RIGHT_STEER.id, FRONT_RIGHT_CANCODER.id },
            { BACK_LEFT_DRIVE.id, BACK_LEFT_STEER.id, BACK_LEFT_CANCODER.id },
            { BACK_RIGHT_DRIVE.id, BACK_RIGHT_STEER.id, BACK_RIGHT_CANCODER.id }
    };

    /** Module names by index */
    public static final String[] MODULE_NAMES = { "Front Left", "Front Right", "Back Left", "Back Right" };
}
