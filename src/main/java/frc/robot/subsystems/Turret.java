package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.util.PhoenixUtil;

@Logged
public class Turret extends SubsystemBase {
    private final TalonFX motor = new TalonFX(TurretConstants.MOTOR_ID, RobotConstants.MECHANISM_BUS);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    @Logged(importance = Logged.Importance.CRITICAL) private double positionRot = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double setpointRot = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean wrappedLastSetpoint = false;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean forwardSoftLimitFault = false;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean reverseSoftLimitFault = false;
    @Logged(importance = Logged.Importance.DEBUG) private double velocityRps = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double supplyCurrentAmps = 0.0;

    public Turret() {
        PhoenixUtil.applyConfig(motor, TurretConstants.CONFIG, "turret");
        zero();
        motor.optimizeBusUtilization();
        motor.getPosition().setUpdateFrequency(50);
        motor.getVelocity().setUpdateFrequency(50);
        motor.getSupplyCurrent().setUpdateFrequency(4);
        motor.getFault_ForwardSoftLimit().setUpdateFrequency(4);
        motor.getFault_ReverseSoftLimit().setUpdateFrequency(4);
    }

    @Override
    public void periodic() {
        positionRot = motor.getPosition().getValueAsDouble();
        velocityRps = motor.getVelocity().getValueAsDouble();
        supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        forwardSoftLimitFault = motor.getFault_ForwardSoftLimit().getValue();
        reverseSoftLimitFault = motor.getFault_ReverseSoftLimit().getValue();
    }

    /**
     * Wraps a setpoint outside the travel limits by one rotation (the turret cannot spin
     * continuously), clamps it, commands it, and returns the value actually used.
     */
    public double setSetpoint(double rot) {
        setpointRot = wrapAndClamp(rot);
        motor.setControl(positionRequest.withPosition(setpointRot));
        return setpointRot;
    }

    private double wrapAndClamp(double rot) {
        double wrapped = rot;
        if (rot < TurretConstants.REVERSE_LIMIT_ROT) {
            wrapped += 1.0;
        } else if (rot > TurretConstants.FORWARD_LIMIT_ROT) {
            wrapped -= 1.0;
        }
        wrappedLastSetpoint = wrapped != rot;
        return MathUtil.clamp(wrapped, TurretConstants.REVERSE_LIMIT_ROT, TurretConstants.FORWARD_LIMIT_ROT);
    }

    /** Declares the current physical position to be straight forward. */
    public void zero() {
        motor.setPosition(0);
    }

    public double getPositionRot() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getSetpointRot() {
        return setpointRot;
    }

    public boolean atSetpoint() {
        return Math.abs(getPositionRot() - setpointRot) < TurretConstants.TOLERANCE_ROT;
    }
}
