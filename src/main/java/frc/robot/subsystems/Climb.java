package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

/**
 * Climb subsystem with automatic telemetry via Epilogue.
 * 
 * <p>
 * The @Logged annotation on TalonFX motors enables automatic logging of
 * motor health data (current, voltage, temperature) via TalonFXLogger.
 */
@Logged
public class Climb extends SubsystemBase {
    private final TalonFX climbMotor;

    private final VoltageOut manualRequest;

    @Logged private double commandedVolts;
    @Logged private double leaderCurrent;
    @Logged private double leaderVoltage;
    @Logged private double leaderPosition;

    public Climb(CANBus canBus) {
        climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID, canBus);

        configureClimbMotor();

        manualRequest = new VoltageOut(0);

        System.out.println("[CLIMB] Subsystem initialized on bus: " + canBus.getName()
                + " | Motor ID=" + ClimbConstants.CLIMB_MOTOR_ID);
    }

    @Override
    public void periodic() {
        leaderCurrent = climbMotor.getSupplyCurrent().getValueAsDouble();
        leaderVoltage = climbMotor.getMotorVoltage().getValueAsDouble();
        leaderPosition = climbMotor.getPosition().getValueAsDouble();
    }

    private void configureClimbMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = climbMotor.getConfigurator().apply(ClimbConstants.CLIMB_CONFIG, 0.1);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply climb motor configs: " + status.toString());
        }
    }


    public void setClimbVoltage(double volts) {
        commandedVolts = volts;
        climbMotor.setControl(manualRequest.withOutput(volts));
    }

    // MOTOR ACCESSORS (for FaultMonitor registration)

    public TalonFX getClimbMotor() {
        return climbMotor;
    }

    public void stop() {
        climbMotor.stopMotor();
    }
}
