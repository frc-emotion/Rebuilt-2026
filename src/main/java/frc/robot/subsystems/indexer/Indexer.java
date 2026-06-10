package frc.robot.subsystems.indexer;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;
import frc.robot.subsystems.indexer.IndexerIO.Stage;

/**
 * Three dumb velocity wheels. All feed decisions (spin-up gating, idle feed, unjam mix, clearing)
 * live in the Superstructure. There is no game-piece sensing on this robot.
 */
@Logged
public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private IndexerIOInputs inputs = IndexerIOInputs.kEmpty;

  @Logged(importance = Logged.Importance.DEBUG)
  private double horizontalVelocityRps = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double verticalVelocityRps = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double upwardVelocityRps = 0.0;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    inputs = io.updateInputs();
    horizontalVelocityRps = inputs.horizontalVelocityRps();
    verticalVelocityRps = inputs.verticalVelocityRps();
    upwardVelocityRps = inputs.upwardVelocityRps();
  }

  public void setVelocity(Stage stage, double rps) {
    io.setVelocity(stage, rps);
  }

  public void stop(Stage stage) {
    io.stop(stage);
  }

  public void stopAll() {
    io.stop(Stage.HORIZONTAL);
    io.stop(Stage.VERTICAL);
    io.stop(Stage.UPWARD);
  }
}
