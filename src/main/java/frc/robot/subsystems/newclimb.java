package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
public class newclimb extends SubsystemBase {
    // Motors marked @Logged will have health data auto-logged via TalonFXLogger
    @Logged
    private final TalonFX leaderMotor;
    @Logged
    private final TalonFX followerMotor;


    private final MotionMagicVoltage leaderMotionRequest;

    public newclimb() {
        leaderMotor = new TalonFX(ClimbConstants.CLIMB_LEADER_ID);
        followerMotor = new TalonFX(ClimbConstants.CLIMB_LEADER_ID);

        configureLeaderMotor();
        configureFollowerMotor();


        leaderMotionRequest = new MotionMagicVoltage(0);



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


    
    public void setClimbPosition(Angle setpoint) {
        // passing in angle as degrees
        leaderMotor.setControl(leaderMotionRequest.withPosition(setpoint).withSlot(0));
    }


    // ==================
    // MOTOR ACCESSORS (for FaultMonitor registration)
    // ==================

    public TalonFX getLeaderMotor() {
        return leaderMotor;
    }

    public TalonFX getfollowerMotor() {
        return followerMotor;
    }
}
