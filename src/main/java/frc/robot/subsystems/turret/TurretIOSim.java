package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;

/**
 * DCMotorSim-backed implementation with the soft limits enforced by clamping the sim position at
 * the legacy thresholds (fault flags report when pinned, like the firmware would).
 */
public class TurretIOSim implements TurretIO {
  // Sim-only physics guess, NOT a tuned robot value.
  private static final double kMoiKgM2 = 0.05;
  private static final DCMotor kMotor = DCMotor.getKrakenX60(1);

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(kMotor, kMoiKgM2, TurretConstants.kGearRatio), kMotor);

  private double zeroOffsetRot = 0.0;
  private double targetRot = 0.0;
  private double feedforwardVolts = 0.0;
  private boolean closedLoop = false;
  private double manualVolts = 0.0;
  private boolean pinnedForward = false;
  private boolean pinnedReverse = false;

  @Override
  public TurretIOInputs updateInputs() {
    double positionRot = sim.getAngularPositionRotations() - zeroOffsetRot;
    double volts;
    if (closedLoop) {
      double error = targetRot - positionRot;
      volts =
          MathUtil.clamp(
              TurretConstants.kConfig.Slot0.kS * Math.signum(error)
                  + TurretConstants.kConfig.Slot0.kP * error
                  + feedforwardVolts,
              -10.0,
              10.0);
    } else {
      volts = manualVolts;
    }

    // Firmware soft limits: refuse to drive further past a limit.
    pinnedForward = positionRot >= TurretConstants.kForwardLimitRot && volts > 0.0;
    pinnedReverse = positionRot <= TurretConstants.kReverseLimitRot && volts < 0.0;
    if (pinnedForward || pinnedReverse) {
      volts = 0.0;
    }

    sim.setInputVoltage(volts);
    sim.update(RobotConstants.kLoopPeriodSeconds);

    return new TurretIOInputs(
        sim.getAngularPositionRotations() - zeroOffsetRot,
        sim.getAngularVelocityRPM() / 60.0,
        sim.getCurrentDrawAmps(),
        volts,
        sim.getAngularPositionRotations(),
        pinnedForward,
        pinnedReverse);
  }

  @Override
  public void setTargetPosition(double rot, double ffVolts) {
    targetRot = rot;
    feedforwardVolts = ffVolts;
    closedLoop = true;
  }

  @Override
  public void setVoltage(double volts) {
    manualVolts = volts;
    closedLoop = false;
  }

  @Override
  public void zeroAtCurrentPosition() {
    zeroOffsetRot = sim.getAngularPositionRotations();
  }

  @Override
  public void stop() {
    manualVolts = 0.0;
    closedLoop = false;
  }
}
