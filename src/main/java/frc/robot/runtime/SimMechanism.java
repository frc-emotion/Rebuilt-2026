package frc.robot.runtime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants;
import frc.robot.runtime.config.MechanismConfig;
import frc.robot.runtime.config.MechanismConfig.Sim;

/**
 * Physics-sim implementation of one mechanism. Mirrors the legacy {@code *IOSim} controllers
 * exactly (same plant, same hand-rolled feedforward+P voltage built from the same gains, same ±peak
 * clamps, same soft-limit pinning) so the standalone sim behaves as it did before the migration.
 * All sims use a single Kraken X60, as the legacy sims did.
 */
final class SimMechanism implements MechanismHandle {
  private static final DCMotor kMotor = DCMotor.getKrakenX60(1);

  private final MechanismConfig config;
  private final Sim sim;

  // Exactly one of these is non-null, chosen by the configured plant.
  private final FlywheelSim flywheel;
  private final DCMotorSim dcMotor;
  private final SingleJointedArmSim arm;

  // Velocity-control state.
  private double setpointRps = 0.0;
  private boolean stopped = true;

  // Position-control state.
  private double targetRot;
  private double feedforwardVolts = 0.0;
  private boolean closedLoop = false;
  private double manualVolts = 0.0;
  private double zeroOffsetRot = 0.0;

  private MechanismState state = MechanismState.kZero;

  SimMechanism(MechanismConfig config) {
    this.config = config;
    this.sim = config.sim();
    this.targetRot = sim.armStartRot();

    switch (sim.plant()) {
      case FLYWHEEL -> {
        flywheel =
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(kMotor, sim.moi(), sim.gearing()), kMotor);
        dcMotor = null;
        arm = null;
      }
      case DCMOTOR -> {
        flywheel = null;
        dcMotor =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(kMotor, sim.moi(), sim.gearing()), kMotor);
        arm = null;
      }
      case ARM -> {
        flywheel = null;
        dcMotor = null;
        arm =
            new SingleJointedArmSim(
                kMotor,
                sim.gearing(),
                sim.moi(),
                sim.armLengthMeters(),
                Units.rotationsToRadians(sim.armMinRot()),
                Units.rotationsToRadians(sim.armMaxRot()),
                sim.gravity(),
                Units.rotationsToRadians(sim.armStartRot()));
      }
      default -> throw new IllegalStateException("unhandled plant " + sim.plant());
    }
  }

  @Override
  public void updateInputs() {
    if (sim.controller() == Sim.Controller.VELOCITY) {
      updateVelocity();
    } else {
      updatePosition();
    }
  }

  private void updateVelocity() {
    double velocityRps = velocityRps();
    double volts = 0.0;
    if (!stopped) {
      volts =
          MathUtil.clamp(
              config.gains().kS() * Math.signum(setpointRps)
                  + config.gains().kV() * setpointRps
                  + config.gains().kP() * (setpointRps - velocityRps),
              config.peakReverseVoltage(),
              config.peakForwardVoltage());
    }
    setInputVoltage(volts);
    update();
    state =
        new MechanismState(positionRot(), velocityRps(), currentAmps(), volts, 0.0, false, false);
  }

  private void updatePosition() {
    double pos = positionRot();
    double volts;
    if (closedLoop) {
      double error = targetRot - pos;
      double ks = sim.useKs() ? config.gains().kS() * Math.signum(error) : 0.0;
      double ff = sim.useFeedforward() ? feedforwardVolts : 0.0;
      volts =
          MathUtil.clamp(
              ks + config.gains().kP() * error + ff,
              config.peakReverseVoltage(),
              config.peakForwardVoltage());
    } else {
      volts = manualVolts;
    }

    boolean pinnedForward = false;
    boolean pinnedReverse = false;
    if (sim.pinSoftLimits() && config.softLimits().isPresent()) {
      MechanismConfig.SoftLimits s = config.softLimits().get();
      pinnedForward = pos >= s.forward() && volts > 0.0;
      pinnedReverse = pos <= s.reverse() && volts < 0.0;
      if (pinnedForward || pinnedReverse) {
        volts = 0.0;
      }
    }

    setInputVoltage(volts);
    update();
    state =
        new MechanismState(
            positionRot(),
            velocityRps(),
            currentAmps(),
            volts,
            rawPositionRot(),
            pinnedForward,
            pinnedReverse);
  }

  // ── Plant adapters ─────────────────────────────────────────────────────

  private double velocityRps() {
    if (flywheel != null) {
      return flywheel.getAngularVelocityRPM() / 60.0;
    }
    if (dcMotor != null) {
      return dcMotor.getAngularVelocityRPM() / 60.0;
    }
    return Units.radiansToRotations(arm.getVelocityRadPerSec());
  }

  private double rawPositionRot() {
    if (dcMotor != null) {
      return dcMotor.getAngularPositionRotations();
    }
    if (arm != null) {
      return Units.radiansToRotations(arm.getAngleRads());
    }
    return 0.0; // flywheel has no position
  }

  private double positionRot() {
    return rawPositionRot() - zeroOffsetRot;
  }

  private double currentAmps() {
    if (flywheel != null) {
      return flywheel.getCurrentDrawAmps();
    }
    if (dcMotor != null) {
      return dcMotor.getCurrentDrawAmps();
    }
    return arm.getCurrentDrawAmps();
  }

  private void setInputVoltage(double volts) {
    if (flywheel != null) {
      flywheel.setInputVoltage(volts);
    } else if (dcMotor != null) {
      dcMotor.setInputVoltage(volts);
    } else {
      arm.setInputVoltage(volts);
    }
  }

  private void update() {
    double dt = RobotConstants.kLoopPeriodSeconds;
    if (flywheel != null) {
      flywheel.update(dt);
    } else if (dcMotor != null) {
      dcMotor.update(dt);
    } else {
      arm.update(dt);
    }
  }

  @Override
  public MechanismState state() {
    return state;
  }

  @Override
  public void applyVelocity(double rps) {
    setpointRps = rps;
    stopped = false;
  }

  @Override
  public void applyPosition(double rot, double ffVolts) {
    targetRot = rot;
    feedforwardVolts = ffVolts;
    closedLoop = true;
  }

  @Override
  public void applyVoltage(double volts) {
    manualVolts = volts;
    closedLoop = false;
  }

  @Override
  public void neutral() {
    setpointRps = 0.0;
    stopped = true;
    manualVolts = 0.0;
    closedLoop = false;
  }

  @Override
  public void zero() {
    zeroOffsetRot = rawPositionRot();
  }

  @Override
  public MechanismConfig config() {
    return config;
  }
}
