package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIO.Stage;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

/**
 * Interp-table calibration workflow (legacy CalibrationShootCommand, verbatim semantics): hold the
 * binding, type hood/RPS values into /Calibration on Elastic until the ball scores, read the echoed
 * live distance, transcribe the triple into ShotCalculator's tables.
 *
 * <p>Turret is pinned to 0, all indexers run UNGATED (unlike match shooting). The binding stays
 * commented out in RobotContainer except during calibration sessions. While held, the
 * Superstructure must be in manual mode so it does not fight this command's setpoints.
 */
public class CalibrationCommand extends Command {
  private final Turret turret;
  private final Hood hood;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Vision vision;

  private final DoubleEntry hoodEntry;
  private final DoubleEntry shooterEntry;
  private final DoubleEntry distanceEntry;

  public CalibrationCommand(
      Turret turret, Hood hood, Shooter shooter, Indexer indexer, Vision vision) {
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    this.indexer = indexer;
    this.vision = vision;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Calibration");
    hoodEntry = table.getDoubleTopic("HoodAngleRot").getEntry(0.04);
    shooterEntry = table.getDoubleTopic("ShooterRPS").getEntry(40.0);
    distanceEntry = table.getDoubleTopic("DistanceMeters").getEntry(0.0);

    addRequirements(turret, hood, shooter, indexer);
  }

  @Override
  public void execute() {
    // Echo live vision distance so the operator can record it from Elastic while calibrating.
    distanceEntry.set(vision != null ? vision.getDistanceToHub() : 0.0);

    turret.setTargetPosition(Rotations.of(0.0));
    hood.setAngle(Rotations.of(hoodEntry.get(0.04)));
    shooter.setVelocity(RotationsPerSecond.of(shooterEntry.get(40.0)));

    indexer.setVelocity(Stage.HORIZONTAL, IndexerConstants.kHorizontalSpeedRps);
    indexer.setVelocity(Stage.VERTICAL, IndexerConstants.kVerticalSpeedRps);
    indexer.setVelocity(Stage.UPWARD, IndexerConstants.kUpwardSpeedRps);
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    hood.stop();
    shooter.stop();
    indexer.stopAll();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
