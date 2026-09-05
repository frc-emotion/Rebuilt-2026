package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.PhoenixUtil;

@Logged
public class Shooter extends SubsystemBase {
    private final TalonFX motor = new TalonFX(ShooterConstants.MOTOR_ID, RobotConstants.MECHANISM_BUS);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final NeutralOut coastRequest = new NeutralOut();

    @Logged(importance = Logged.Importance.CRITICAL) private double setpointRps = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double velocityRps = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double supplyCurrentAmps = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double motorVoltage = 0.0;

    public Shooter() {
        PhoenixUtil.applyConfig(motor, ShooterConstants.CONFIG, "shooter");
        motor.optimizeBusUtilization();
        motor.getVelocity().setUpdateFrequency(50);
        motor.getSupplyCurrent().setUpdateFrequency(10);
        motor.getMotorVoltage().setUpdateFrequency(10);
    }

    @Override
    public void periodic() {
        velocityRps = motor.getVelocity().getValueAsDouble();
        supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        motorVoltage = motor.getMotorVoltage().getValueAsDouble();
    }

    public void setVelocity(double rps) {
        setpointRps = MathUtil.clamp(rps, 0.0, ShooterConstants.MAX_RPS);
        motor.setControl(velocityRequest.withVelocity(setpointRps));
    }

    public void coast() {
        setpointRps = 0.0;
        motor.setControl(coastRequest);
    }

    public double getVelocityRps() {
        return velocityRps;
    }

    public double getSetpointRps() {
        return setpointRps;
    }

    public boolean atSetpoint() {
        return Math.abs(velocityRps - setpointRps) < ShooterConstants.TOLERANCE_RPS;
    }
}
