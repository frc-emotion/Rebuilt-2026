package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.util.PhoenixUtil;

@Logged
public class Hood extends SubsystemBase {
    private final TalonFX motor = new TalonFX(HoodConstants.MOTOR_ID, RobotConstants.MECHANISM_BUS);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    @Logged(importance = Logged.Importance.CRITICAL) private double positionRot = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double setpointRot = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double velocityRps = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double supplyCurrentAmps = 0.0;

    public Hood() {
        PhoenixUtil.applyConfig(motor, HoodConstants.CONFIG, "hood");
        zero();
        motor.optimizeBusUtilization();
        motor.getPosition().setUpdateFrequency(50);
        motor.getVelocity().setUpdateFrequency(4);
        motor.getSupplyCurrent().setUpdateFrequency(4);
    }

    @Override
    public void periodic() {
        positionRot = motor.getPosition().getValueAsDouble();
        velocityRps = motor.getVelocity().getValueAsDouble();
        supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    }

    /** Clamps to the hood range, commands it, and returns the value actually used. */
    public double setSetpoint(double rot) {
        setpointRot = MathUtil.clamp(rot, HoodConstants.MIN_ROT, HoodConstants.MAX_ROT);
        motor.setControl(positionRequest.withPosition(setpointRot));
        return setpointRot;
    }

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
        return Math.abs(getPositionRot() - setpointRot) < HoodConstants.TOLERANCE_ROT;
    }
}
