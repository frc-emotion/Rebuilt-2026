package frc.robot.subsystems.hood;

/** Hardware boundary for the hood. The subsystem reads only the inputs record. */
public interface HoodIO {

  record HoodIOInputs(
      double positionRot, double velocityRps, double supplyCurrentAmps, double appliedVolts) {
    public static final HoodIOInputs kEmpty = new HoodIOInputs(0.0, 0.0, 0.0, 0.0);
  }

  /** Read every sensor value once per loop. */
  HoodIOInputs updateInputs();

  /** MotionMagic position, mechanism rotations. */
  void setTargetPosition(double rot);

  /** Open-loop jog for manual mode; firmware soft limits remain active. */
  void setVoltage(double volts);

  void stop();
}
