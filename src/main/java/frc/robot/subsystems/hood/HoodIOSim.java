package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;

/**
 * DCMotorSim-backed implementation (gravity was never compensated in the tuned gains, so an arm sim
 * would model physics the controller was not tuned for). Emulates the position loop with the real
 * kP and output limits; kI is omitted in sim to avoid windup the firmware handles internally.
 */
public class HoodIOSim implements HoodIO {
  // Sim-only physics guesses, NOT tuned robot values.
  private static final double kMoiKgM2 = 0.002;
  private static final DCMotor kMotor = DCMotor.getKrakenX60(1);

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(kMotor, kMoiKgM2, HoodConstants.kGearRatio), kMotor);

  private double targetRot = 0.0;
  private boolean closedLoop = false;
  private double manualVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public HoodIOInputs updateInputs() {
    double positionRot = sim.getAngularPositionRotations();
    if (closedLoop) {
      double error = targetRot - positionRot;
      appliedVolts =
          MathUtil.clamp(
              HoodConstants.kConfig.Slot0.kP * error,
              HoodConstants.kConfig.Voltage.PeakReverseVoltage,
              HoodConstants.kConfig.Voltage.PeakForwardVoltage);
    } else {
      appliedVolts = manualVolts;
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(RobotConstants.kLoopPeriodSeconds);
    return new HoodIOInputs(
        sim.getAngularPositionRotations(),
        sim.getAngularVelocityRPM() / 60.0,
        sim.getCurrentDrawAmps(),
        appliedVolts);
  }

  @Override
  public void setTargetPosition(double rot) {
    targetRot = rot;
    closedLoop = true;
  }

  @Override
  public void setVoltage(double volts) {
    manualVolts = volts;
    closedLoop = false;
  }

  @Override
  public void stop() {
    manualVolts = 0.0;
    closedLoop = false;
  }
}
