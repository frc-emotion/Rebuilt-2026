package frc.robot.subsystems.intake;

/**
 * Hardware boundary for the intake pivot and roller. The subsystem reads only the inputs record.
 */
public interface IntakeIO {

  record IntakeIOInputs(
      double pivotPositionRot, double pivotSupplyCurrentAmps, double rollerVelocityRps) {
    public static final IntakeIOInputs kEmpty = new IntakeIOInputs(0.0, 0.0, 0.0);
  }

  /** Read every sensor value once per loop. */
  IntakeIOInputs updateInputs();

  /** MotionMagic pivot position, mechanism rotations. */
  void setPivotTarget(double rot);

  /** Open-loop pivot jog for manual mode; firmware soft limits remain active. */
  void setPivotVoltage(double volts);

  /** Closed-loop roller velocity, rotations per second. */
  void setRollerVelocity(double rps);

  /** Roller to NeutralOut (coast) — the legacy stop semantics. */
  void stopRoller();

  void stop();
}
