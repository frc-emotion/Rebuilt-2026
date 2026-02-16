package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

/**
 * Intake subsystem with automatic telemetry via Epilogue.
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


    private final MotionMagicVoltage leaderMotionRequest;

    private final VoltageOut manualRequest;


    private double currentSetpoint;

    public Climb(CANBus canBus) {
        leaderMotor = new TalonFX(ClimbConstants.CLIMB_LEADER_ID, canBus);
        followerMotor = new TalonFX(ClimbConstants.CLIMB_LEADER_ID, canBus);

        configureLeaderMotor();
        configureFollowerMotor();


        leaderMotionRequest = new MotionMagicVoltage(0);

        manualRequest = new VoltageOut(0);




    }

    public void periodic() {
    }

    private void configureLeaderMotor() {

        leaderMotor.getConfigurator().apply(ClimbConstants.CLIMB_CONFIG);

    }

    private void configureFollowerMotor() {

        followerMotor.getConfigurator().apply(ClimbConstants.CLIMB_CONFIG);
        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));
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
