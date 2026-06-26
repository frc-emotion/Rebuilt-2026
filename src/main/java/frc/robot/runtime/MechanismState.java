package frc.robot.runtime;

/**
 * One mechanism's sensor reads for a single loop — the generic equivalent of the per-subsystem
 * {@code *IOInputs} records, read exactly once per loop. Position/velocity fields are zero for
 * mechanisms that do not have them (e.g. a flywheel reports no position). Auxiliary fields
 * (cancoder, soft-limit faults) are zero/false when the mechanism has no such sensor.
 */
public record MechanismState(
    double positionRot,
    double velocityRps,
    double supplyCurrentAmps,
    double appliedVolts,
    double cancoderAbsoluteRot,
    boolean faultForwardSoftLimit,
    boolean faultReverseSoftLimit) {

  public static final MechanismState kZero =
      new MechanismState(0.0, 0.0, 0.0, 0.0, 0.0, false, false);
}
