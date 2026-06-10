package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotConstants;

/**
 * FlywheelSim-backed implementation. Mirrors the real controller's velocity loop (kS/kV/kP from
 * ShooterConstants.kConfig) with the real 12V-forward / 0V-reverse output limits.
 */
public class ShooterIOSim implements ShooterIO {
  // Sim-only physics guesses, NOT tuned robot values.
  private static final double kFlywheelMoiKgM2 = 0.004;
  private static final DCMotor kMotor = DCMotor.getKrakenX60(1);

  private final FlywheelSim sim =
      new FlywheelSim(LinearSystemId.createFlywheelSystem(kMotor, kFlywheelMoiKgM2, 1.0), kMotor);

  private double setpointRps = 0.0;
  private boolean stopped = true;
  private double appliedVolts = 0.0;

  @Override
  public ShooterIOInputs updateInputs() {
    double velocityRps = sim.getAngularVelocityRPM() / 60.0;
    if (stopped) {
      appliedVolts = 0.0;
    } else {
      double error = setpointRps - velocityRps;
      double rawVolts =
          ShooterConstants.kConfig.Slot0.kS * Math.signum(setpointRps)
              + ShooterConstants.kConfig.Slot0.kV * setpointRps
              + ShooterConstants.kConfig.Slot0.kP * error;
      appliedVolts =
          MathUtil.clamp(
              rawVolts,
              ShooterConstants.kConfig.Voltage.PeakReverseVoltage,
              ShooterConstants.kConfig.Voltage.PeakForwardVoltage);
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(RobotConstants.kLoopPeriodSeconds);
    return new ShooterIOInputs(
        sim.getAngularVelocityRPM() / 60.0, sim.getCurrentDrawAmps(), appliedVolts);
  }

  @Override
  public void setVelocity(double rps) {
    setpointRps = rps;
    stopped = false;
  }

  @Override
  public void stop() {
    setpointRps = 0.0;
    stopped = true;
  }
}
