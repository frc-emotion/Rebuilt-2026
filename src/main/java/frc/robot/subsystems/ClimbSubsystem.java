package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;

    // Controls
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // Signals for telemetry
    private final StatusSignal<Angle> leaderPosition;
    private final StatusSignal<Voltage> leaderVoltage;

    public ClimbSubsystem() {
        leaderMotor = new TalonFX(ClimbConstants.CLIMB_LEADER_ID);
        followerMotor = new TalonFX(ClimbConstants.CLIMB_FOLLOWER_ID);

        configureMotors();

        leaderPosition = leaderMotor.getPosition();
        leaderVoltage = leaderMotor.getMotorVoltage();

        // Optimize bus usage
        leaderPosition.setUpdateFrequency(50);
        leaderVoltage.setUpdateFrequency(50);
    }

    private void configureMotors() {
        // Apply config to leader
        leaderMotor.getConfigurator().apply(ClimbConstants.CLIMB_CONFIG);
        
        // Apply config to follower (mostly for current limits/brake mode)
        followerMotor.getConfigurator().apply(ClimbConstants.CLIMB_CONFIG);
        
        // Set follower to strictly follow leader (false = same direction)
        followerMotor.setControl(new Follower(ClimbConstants.CLIMB_LEADER_ID, false));
    }

    @Override
    public void periodic() {
        leaderPosition.refresh();
        leaderVoltage.refresh();

        SmartDashboard.putNumber("Climb/Position", leaderPosition.getValueAsDouble());
        SmartDashboard.putNumber("Climb/Voltage", leaderVoltage.getValueAsDouble());
    }

    /**
     * Moves the climb to a specific position using Motion Magic
     * @param rotations Target position in motor rotations
     */
    public void setPosition(double rotations) {
        leaderMotor.setControl(positionRequest.withPosition(rotations));
    }

    /**
     * Manual control using voltage
     * @param volts Voltage to apply
     */
    public void setVoltage(double volts) {
        leaderMotor.setControl(voltageRequest.withOutput(volts));
    }

    /**
     * Resets internal encoder to zero (useful if starting deployed)
     */
    public void zeroSensor() {
        leaderMotor.setPosition(0);
    }

    /**
     * Check if we are near a target position
     */
    public boolean atSetpoint(double targetRotations, double tolerance) {
        return Math.abs(leaderPosition.getValueAsDouble() - targetRotations) < tolerance;
    }

    public TalonFX getLeaderMotor() {
        return leaderMotor;
    }

    public TalonFX getFollowerMotor() {
        return followerMotor;
    }
}