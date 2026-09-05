package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.util.PhoenixUtil;

@Logged
public class Intake extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, RobotConstants.MECHANISM_BUS);
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, RobotConstants.MECHANISM_BUS);
    private final CANcoder pivotEncoder = new CANcoder(IntakeConstants.PIVOT_ENCODER_ID, RobotConstants.MECHANISM_BUS);

    private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);
    private final VelocityVoltage rollerRequest = new VelocityVoltage(0);
    private final NeutralOut rollerStopRequest = new NeutralOut();

    @Logged(importance = Logged.Importance.CRITICAL) private double pivotPositionRot = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double pivotSetpointRot = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double rollerVelocityRps = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double pivotSupplyCurrentAmps = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double encoderAbsolutePositionRot = 0.0;

    public Intake() {
        PhoenixUtil.applyConfig(pivotEncoder, IntakeConstants.PIVOT_ENCODER_CONFIG, "intake pivot encoder");
        PhoenixUtil.applyConfig(pivotMotor, IntakeConstants.PIVOT_CONFIG, "intake pivot");
        PhoenixUtil.applyConfig(rollerMotor, IntakeConstants.ROLLER_CONFIG, "intake roller");

        ParentDevice.optimizeBusUtilizationForAll(pivotMotor, rollerMotor, pivotEncoder);
        pivotEncoder.getPosition().setUpdateFrequency(100);
        pivotEncoder.getAbsolutePosition().setUpdateFrequency(4);
        pivotMotor.getPosition().setUpdateFrequency(50);
        pivotMotor.getSupplyCurrent().setUpdateFrequency(4);
        rollerMotor.getVelocity().setUpdateFrequency(10);
    }

    @Override
    public void periodic() {
        pivotPositionRot = pivotMotor.getPosition().getValueAsDouble();
        rollerVelocityRps = rollerMotor.getVelocity().getValueAsDouble();
        pivotSupplyCurrentAmps = pivotMotor.getSupplyCurrent().getValueAsDouble();
        encoderAbsolutePositionRot = pivotEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setDeployed(boolean out) {
        pivotSetpointRot = out ? IntakeConstants.OUT_POSITION_ROT : IntakeConstants.STOWED_POSITION_ROT;
        pivotMotor.setControl(pivotRequest.withPosition(pivotSetpointRot));
    }

    public void runRoller() {
        rollerMotor.setControl(rollerRequest.withVelocity(IntakeConstants.ROLLER_SPEED_RPS));
    }

    public void stopRoller() {
        rollerMotor.setControl(rollerStopRequest);
    }

    public double getPositionRot() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    /** True inside the wide tolerance band around the out position, regardless of where it was told to go. */
    public boolean isOut() {
        return Math.abs(getPositionRot() - IntakeConstants.OUT_POSITION_ROT) < IntakeConstants.OUT_TOLERANCE_ROT;
    }
}
