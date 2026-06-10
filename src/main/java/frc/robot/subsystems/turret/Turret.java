package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
import frc.robot.subsystems.turret.TurretWrap.WrapResult;

/**
 * Dumb position executor for the turret azimuth. The wrap (W1) is the turret's own safety
 * behavior; everything about WHERE to aim lives in the Superstructure's TurretAiming helper.
 *
 * <p>No nested state machine, deliberately: the apparent turret states (tracking/passing/manual)
 * were robot-level aiming decisions all along and live in RobotState. Wrap is stateless math per
 * command and there is no homing sequence (boot-zero).
 */
@Logged
public class Turret extends SubsystemBase {
  private final TurretIO io;
  private TurretIOInputs inputs = TurretIOInputs.kEmpty;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double positionRot = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double setpointRot = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private double errorRot = 0.0;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean wrapped = false;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean faultForwardSoftLimit = false;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean faultReverseSoftLimit = false;

  @Logged(importance = Logged.Importance.DEBUG)
  private double velocityRps = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double supplyCurrentAmps = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double appliedVolts = 0.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double cancoderAbsoluteRot = 0.0;

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    inputs = io.updateInputs();
    positionRot = inputs.positionRot();
    errorRot = setpointRot - positionRot;
    velocityRps = inputs.velocityRps();
    supplyCurrentAmps = inputs.supplyCurrentAmps();
    appliedVolts = inputs.appliedVolts();
    cancoderAbsoluteRot = inputs.cancoderAbsoluteRot();
    faultForwardSoftLimit = inputs.faultForwardSoftLimit();
    faultReverseSoftLimit = inputs.faultReverseSoftLimit();
  }

  /**
   * Wraps and clamps the request, commands MotionMagic, and returns the ACTUALLY-COMMANDED angle.
   * Callers integrating a target (gyro feedforward) must store the returned value back (W2) or
   * their accumulator winds up unboundedly while the mechanism sits clamped.
   */
  public Angle setTargetPosition(Angle setpoint) {
    return setTargetPosition(setpoint, 0.0);
  }

  public Angle setTargetPosition(Angle setpoint, double feedforwardVolts) {
    WrapResult result =
        TurretWrap.apply(
            setpoint.in(Rotations), TurretConstants.kReverseLimitRot, TurretConstants.kForwardLimitRot);
    setpointRot = result.commandedRot();
    wrapped = result.wrapped();
    io.setTargetPosition(setpointRot, feedforwardVolts);
    return Rotations.of(setpointRot);
  }

  /** Manual jog: joystick input × 3.0 V (legacy scale); firmware soft limits stay active. */
  public void setManualVoltage(double joystickInput) {
    io.setVoltage(MathUtil.clamp(joystickInput, -1, 1) * TurretConstants.kManualVoltsPerUnit);
  }

  /** Redefines the current physical position as zero (boot procedure + operator RB re-zero). */
  public void zeroAtCurrentPosition() {
    io.zeroAtCurrentPosition();
    setpointRot = 0.0;
  }

  public boolean atSetpoint() {
    return Math.abs(positionRot - setpointRot) < TurretConstants.kToleranceRot;
  }

  public Rotation2d getPosition() {
    return Rotation2d.fromRotations(positionRot);
  }

  public double getPositionRot() {
    return positionRot;
  }

  public double getVelocityRps() {
    return velocityRps;
  }

  public void stop() {
    io.stop();
  }
}
