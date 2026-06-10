package frc.robot.subsystems.turret;

/** Hardware boundary for the turret azimuth. The subsystem reads only the inputs record. */
public interface TurretIO {

  record TurretIOInputs(
      double positionRot,
      double velocityRps,
      double supplyCurrentAmps,
      double appliedVolts,
      double cancoderAbsoluteRot,
      boolean faultForwardSoftLimit,
      boolean faultReverseSoftLimit) {
    public static final TurretIOInputs kEmpty =
        new TurretIOInputs(0.0, 0.0, 0.0, 0.0, 0.0, false, false);
  }

  /** Read every sensor value once per loop. */
  TurretIOInputs updateInputs();

  /** MotionMagic position (mechanism rotations) with an arbitrary feedforward in volts. */
  void setTargetPosition(double rot, double feedforwardVolts);

  /** Open-loop manual jog; firmware soft limits remain active. */
  void setVoltage(double volts);

  /** Redefine the current physical position as zero (boot zeroing + operator RB re-zero). */
  void zeroAtCurrentPosition();

  void stop();
}
