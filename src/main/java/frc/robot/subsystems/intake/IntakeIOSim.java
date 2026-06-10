package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants;

/**
 * SingleJointedArmSim pivot (gravity is exactly why over-travel recovery exists — the sim must
 * reproduce the failure mode) plus a DCMotorSim roller.
 */
public class IntakeIOSim implements IntakeIO {
  // Sim-only physics guesses, NOT tuned robot values.
  private static final DCMotor kPivotMotor = DCMotor.getKrakenX60(1);
  private static final DCMotor kRollerMotor = DCMotor.getKrakenX60(1);
  private static final double kArmLengthMeters = 0.35;
  private static final double kArmMoiKgM2 = 0.3;
  private static final double kRollerMoiKgM2 = 0.001;

  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          kPivotMotor,
          27.0,
          kArmMoiKgM2,
          kArmLengthMeters,
          Units.rotationsToRadians(IntakeConstants.kReverseSoftLimitRot),
          Units.rotationsToRadians(IntakeConstants.kForwardSoftLimitRot),
          true,
          Units.rotationsToRadians(IntakeConstants.kInAngle.in(Rotations)));

  private final DCMotorSim rollerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(kRollerMotor, kRollerMoiKgM2, 1.0), kRollerMotor);

  private double pivotTargetRot = IntakeConstants.kInAngle.in(Rotations);
  private boolean pivotClosedLoop = true;
  private double pivotManualVolts = 0.0;
  private double rollerSetpointRps = 0.0;
  private boolean rollerStopped = true;

  @Override
  public IntakeIOInputs updateInputs() {
    double pivotPositionRot = Units.radiansToRotations(pivotSim.getAngleRads());
    double pivotVolts;
    if (pivotClosedLoop) {
      double error = pivotTargetRot - pivotPositionRot;
      pivotVolts = MathUtil.clamp(IntakeConstants.kPivotConfig.Slot0.kP * error, -10.0, 10.0);
    } else {
      pivotVolts = pivotManualVolts;
    }
    pivotSim.setInputVoltage(pivotVolts);
    pivotSim.update(RobotConstants.kLoopPeriodSeconds);

    double rollerVolts = 0.0;
    if (!rollerStopped) {
      double rollerVelocityRps = rollerSim.getAngularVelocityRPM() / 60.0;
      double error = rollerSetpointRps - rollerVelocityRps;
      rollerVolts =
          MathUtil.clamp(
              0.15 * Math.signum(rollerSetpointRps) + 0.12 * rollerSetpointRps + 0.3 * error,
              -10.0,
              10.0);
    }
    rollerSim.setInputVoltage(rollerVolts);
    rollerSim.update(RobotConstants.kLoopPeriodSeconds);

    return new IntakeIOInputs(
        Units.radiansToRotations(pivotSim.getAngleRads()),
        pivotSim.getCurrentDrawAmps(),
        rollerSim.getAngularVelocityRPM() / 60.0);
  }

  @Override
  public void setPivotTarget(double rot) {
    pivotTargetRot = rot;
    pivotClosedLoop = true;
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotManualVolts = volts;
    pivotClosedLoop = false;
  }

  @Override
  public void setRollerVelocity(double rps) {
    rollerSetpointRps = rps;
    rollerStopped = false;
  }

  @Override
  public void stopRoller() {
    rollerSetpointRps = 0.0;
    rollerStopped = true;
  }

  @Override
  public void stop() {
    pivotClosedLoop = false;
    pivotManualVolts = 0.0;
    stopRoller();
  }
}
