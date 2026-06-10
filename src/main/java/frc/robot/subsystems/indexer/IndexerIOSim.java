package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;

/** Three DCMotorSim velocity wheels mirroring the shared kS/kV/kP gains and ±10 V limits. */
public class IndexerIOSim implements IndexerIO {
  // Sim-only physics guess, NOT a tuned robot value.
  private static final double kMoiKgM2 = 0.001;
  private static final DCMotor kMotor = DCMotor.getKrakenX60(1);

  private final DCMotorSim[] sims = new DCMotorSim[Stage.values().length];
  private final double[] setpointsRps = new double[Stage.values().length];
  private final boolean[] stopped = {true, true, true};

  public IndexerIOSim() {
    for (int i = 0; i < sims.length; i++) {
      sims[i] = new DCMotorSim(LinearSystemId.createDCMotorSystem(kMotor, kMoiKgM2, 1.0), kMotor);
    }
  }

  @Override
  public IndexerIOInputs updateInputs() {
    for (Stage stage : Stage.values()) {
      int i = stage.ordinal();
      double velocityRps = sims[i].getAngularVelocityRPM() / 60.0;
      double volts = 0.0;
      if (!stopped[i]) {
        // All three stages share gains; mirror the vertical config so retunes propagate.
        com.ctre.phoenix6.configs.Slot0Configs slot0 = IndexerConstants.kVerticalConfig.Slot0;
        double error = setpointsRps[i] - velocityRps;
        volts =
            MathUtil.clamp(
                slot0.kS * Math.signum(setpointsRps[i]) + slot0.kV * setpointsRps[i] + slot0.kP * error,
                IndexerConstants.kVerticalConfig.Voltage.PeakReverseVoltage,
                IndexerConstants.kVerticalConfig.Voltage.PeakForwardVoltage);
      }
      sims[i].setInputVoltage(volts);
      sims[i].update(RobotConstants.kLoopPeriodSeconds);
    }
    return new IndexerIOInputs(
        sims[Stage.HORIZONTAL.ordinal()].getAngularVelocityRPM() / 60.0,
        sims[Stage.VERTICAL.ordinal()].getAngularVelocityRPM() / 60.0,
        sims[Stage.UPWARD.ordinal()].getAngularVelocityRPM() / 60.0);
  }

  @Override
  public void setVelocity(Stage stage, double rps) {
    setpointsRps[stage.ordinal()] = rps;
    stopped[stage.ordinal()] = false;
  }

  @Override
  public void stop(Stage stage) {
    setpointsRps[stage.ordinal()] = 0.0;
    stopped[stage.ordinal()] = true;
  }
}
