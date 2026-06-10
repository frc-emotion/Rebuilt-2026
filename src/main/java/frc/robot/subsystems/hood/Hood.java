package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;

/** Dumb position executor for the hood. All decisions live in the Superstructure. */
@Logged
public class Hood extends SubsystemBase {
  private final HoodIO io;
  private HoodIOInputs inputs = HoodIOInputs.kEmpty;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double positionRot = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double setpointRot = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double velocityRps = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double supplyCurrentAmps = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double appliedVolts = 0.0;

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    inputs = io.updateInputs();
    positionRot = inputs.positionRot();
    velocityRps = inputs.velocityRps();
    supplyCurrentAmps = inputs.supplyCurrentAmps();
    appliedVolts = inputs.appliedVolts();
  }

  /** Clamped to the hood's physical travel [0.0, 0.08] rot (legacy clamp, verbatim). */
  public void setAngle(Angle angle) {
    setpointRot =
        MathUtil.clamp(
            angle.in(Rotations),
            HoodConstants.kReverseHardStopRot,
            HoodConstants.kForwardHardStopRot);
    io.setTargetPosition(setpointRot);
  }

  /** Captures the current position ONCE and holds it (replaces the legacy sag-follow, W32). */
  public void holdCurrentPosition() {
    setAngle(Rotations.of(positionRot));
  }

  /** Open-loop jog for manual mode; firmware soft limits remain the safety net. */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
    setpointRot = positionRot;
  }

  public void stop() {
    io.stop();
  }

  public boolean atSetpoint() {
    return Math.abs(positionRot - setpointRot) < HoodConstants.kToleranceRot;
  }

  public double getPositionRot() {
    return positionRot;
  }
}
