package frc.robot;

import com.ctre.phoenix6.CANBus;

/** Cross-subsystem constants only. Per-mechanism values live in each subsystem's constants file. */
public final class RobotConstants {
  private RobotConstants() {}

  public static final double kLoopPeriodSeconds = 0.02;

  public static final int kDriverPort = 0;
  public static final int kOperatorPort = 1;

  // All mechanism motors live on this bus; the swerve drivetrain lives on the CANivore
  // ("Persian  Canivore" — the double space is the real device name, never "fix" it).
  public static final CANBus kMechanismBus = new CANBus("mechanisms");

  // When true, TunableNumbers read live from NetworkTables under Tuning/.
  public static final boolean kTuningMode = false;

  // Feature flags: a disabled mechanism is constructed as null and every binding is guarded.
  public static final boolean kEnableVision = true;
  public static final boolean kEnableIntake = true;
  public static final boolean kEnableIndexer = true;
  public static final boolean kEnableTurret = true;
  public static final boolean kEnableHood = true;
  public static final boolean kEnableShooter = true;
}
