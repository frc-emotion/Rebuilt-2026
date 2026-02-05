package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

/*
 * Climb subsystem 
 */

@Logged
public class Climb extends SubsystemBase {

    // will log the health of the motors, current, temperature, and voltage

    @Logged
    private final TalonFX leadMotor;
    @Logged
    private final TalonFX followerMotor;

    private final DutyCycleOut climbRequest = new DutyCycleOut(0);

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;

    public Climb() {
        leadMotor = new TalonFX(ClimbConstants.climbMotorLeaderID);
        followerMotor = new TalonFX(ClimbConstants.climbMotorFollowerID);
        configureMotors();

        // Telemetry
        position = leadMotor.getPosition();
        velocity = leadMotor.getVelocity();
        voltage = leadMotor.getMotorVoltage();
        current = leadMotor.getSupplyCurrent();

        // updates the values 50 times a second
        position.setUpdateFrequency(50);
        velocity.setUpdateFrequency(50);
        voltage.setUpdateFrequency(50);
        current.setUpdateFrequency(50);
    }

    private void configureMotors() {
        // Apply configuration to lead motor
        leadMotor.getConfigurator().apply(ClimbConstants.CLIMB_CONFIG);

        // Follower motor follows lead motor, same direction
        followerMotor.setControl(new Follower(leadMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        // Apply same config to follower for consistency (neutral mode, limits, etc.)
        followerMotor.getConfigurator().apply(ClimbConstants.CLIMB_CONFIG);
    }

    @Override
    public void periodic() {
        position.refresh();
        velocity.refresh();
        voltage.refresh();
        current.refresh();
    }

    // Sets the climb speed with duty cycle control.
    // speed from -1.0 to 1.0. Positive moves UP (Clockwise).

    public void setClimbSpeed(double speed) {
        leadMotor.setControl(climbRequest.withOutput(speed));
    }

    // Stops the climb motors

    public void stop() {
        leadMotor.stopMotor();
    }

    // Command to move the climb up at a constant speed.

    public Command climbUpCommand() {
        return run(() -> setClimbSpeed(ClimbConstants.CLIMB_UP_SPEED))
                .withName("Climb Up");
    }

    // command to move the climb down at a constant speed.

    public Command climbDownCommand() {
        return run(() -> setClimbSpeed(ClimbConstants.CLIMB_DOWN_SPEED))
                .withName("Climb Down");
    }

    // Command to stop the climb.

    public Command stopClimbCommand() {
        return runOnce(this::stop)
                .withName("Stop Climb");
    }

    // Command to manually control the climb with a provided speed supplier.

    public Command manualClimb(java.util.function.DoubleSupplier speedSupplier) {
        return run(() -> setClimbSpeed(speedSupplier.getAsDouble()))
                .withName("Manual Climb");
    }

    // Accessor methods for motors

    public TalonFX getLeadMotor() {
        return leadMotor;
    }

    public TalonFX getFollowerMotor() {
        return followerMotor;
    }

    // get method for position

    public Angle getPosition() {
        return position.getValue();
    }
}
