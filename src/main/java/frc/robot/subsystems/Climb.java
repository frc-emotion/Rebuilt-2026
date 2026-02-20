package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;

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
    // Motors marked @Logged will have health data auto-logged via TalonFXLogger
    @Logged
    private final TalonFX leaderMotor;
    @Logged
    private final TalonFX followerMotor;
    
    private final CANcoder climbEncoder;

    private final MotionMagicVoltage leaderMotionRequest;

    private final VoltageOut manualRequest;


    private double currentSetpoint;

    public Climb(CANBus canBus) {
        leaderMotor = new TalonFX(ClimbConstants.CLIMB_LEADER_ID, canBus);
        followerMotor = new TalonFX(ClimbConstants.CLIMB_FOLLOWER_ID, canBus);

        climbEncoder = new CANcoder(ClimbConstants.CLIMB_ENCODER_ID, canBus);


        configureLeaderMotor();
        configureFollowerMotor();
        configureClimbEncoder();


        leaderMotionRequest = new MotionMagicVoltage(0);

        manualRequest = new VoltageOut(0);




    }

    public void periodic() {
    }

    private void configureLeaderMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = leaderMotor.getConfigurator().apply(ClimbConstants.CLIMB_CONFIG, 0.1);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply climb leader motor configs: " + status.toString());
        }
    }

    private void configureFollowerMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = followerMotor.getConfigurator().apply(ClimbConstants.CLIMB_CONFIG, 0.1);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply climb follower motor configs: " + status.toString());
        }
        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configureClimbEncoder() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = climbEncoder.getConfigurator().apply(ClimbConstants.CLIMB_ENCODER_CONFIG, 0.1);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply climb encoder configs: " + status.toString());
        }
    }


    
    public void setClimbPosition(double setpoint) {
        currentSetpoint = setpoint;
        // passing in position in rotations
        leaderMotor.setControl(leaderMotionRequest.withPosition(setpoint));
    }


    public boolean atSetpoint(){
        return Math.abs(leaderMotor.getPosition().getValueAsDouble() - currentSetpoint) < ClimbConstants.TOLERANCE;
    }

    public void setManualVoltage(double joystickInput) {
        double manualVolts = joystickInput * 12.0;
        leaderMotor.setControl(manualRequest.withOutput(manualVolts + ClimbConstants.manual_kG));
    }


    // MOTOR ACCESSORS (for FaultMonitor registration)

    public TalonFX getLeaderMotor() {
        return leaderMotor;
    }

    public TalonFX getFollowerMotor() {
        return followerMotor;
    }

    public void stop(){
        leaderMotor.stopMotor();
        followerMotor.stopMotor();
    }
}
