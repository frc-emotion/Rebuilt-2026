package frc.robot.subsystems.indexer;

/** Hardware boundary for the three indexer stages. The subsystem reads only the inputs record. */
public interface IndexerIO {

  /** The three feed stages, in legacy enum order. */
  enum Stage {
    VERTICAL,
    HORIZONTAL,
    UPWARD
  }

  record IndexerIOInputs(
      double horizontalVelocityRps, double verticalVelocityRps, double upwardVelocityRps) {
    public static final IndexerIOInputs kEmpty = new IndexerIOInputs(0.0, 0.0, 0.0);
  }

  /** Read every sensor value once per loop. */
  IndexerIOInputs updateInputs();

  /** Closed-loop velocity, rotations per second; negative reverses. */
  void setVelocity(Stage stage, double rps);

  void stop(Stage stage);
}
