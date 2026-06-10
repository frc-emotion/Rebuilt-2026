package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

/** Dumb velocity executor for the flywheel. All decisions live in the Superstructure. */
@Logged
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private ShooterIOInputs inputs = ShooterIOInputs.kEmpty;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double setpointRps = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double velocityRps = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double supplyCurrentAmps = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double appliedVolts = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    inputs = io.updateInputs();
    velocityRps = inputs.velocityRps();
    supplyCurrentAmps = inputs.supplyCurrentAmps();
    appliedVolts = inputs.appliedVolts();
  }

  /** Clamped to [0, kMaxRps]: the flywheel is never commanded backwards. */
  public void setVelocity(AngularVelocity speed) {
    setpointRps = MathUtil.clamp(speed.in(RotationsPerSecond), 0.0, ShooterConstants.kMaxRps);
    io.setVelocity(setpointRps);
  }

  public void stop() {
    setpointRps = 0.0;
    io.stop();
  }

  public boolean atSpeed() {
    return Math.abs(velocityRps - setpointRps) < ShooterConstants.kToleranceRps;
  }

  public boolean commandedNonZero() {
    return setpointRps > 0.0;
  }
}
