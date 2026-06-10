package frc.robot.subsystems.shooter;

/** Hardware boundary for the shooter flywheel. The subsystem reads only the inputs record. */
public interface ShooterIO {

  record ShooterIOInputs(double velocityRps, double supplyCurrentAmps, double appliedVolts) {
    public static final ShooterIOInputs kEmpty = new ShooterIOInputs(0.0, 0.0, 0.0);
  }

  /** Read every sensor value once per loop. */
  ShooterIOInputs updateInputs();

  /** Closed-loop velocity, rotations per second. */
  void setVelocity(double rps);

  void stop();
}
