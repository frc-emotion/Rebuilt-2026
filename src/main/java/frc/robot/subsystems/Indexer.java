package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants.Stage;
import frc.robot.constants.RobotConstants;
import frc.robot.util.PhoenixUtil;

@Logged
public class Indexer extends SubsystemBase {
    private final Map<Stage, TalonFX> motors = new EnumMap<>(Stage.class);
    private final Map<Stage, VelocityVoltage> velocityRequests = new EnumMap<>(Stage.class);
    private final NeutralOut stopRequest = new NeutralOut();

    @Logged(importance = Logged.Importance.DEBUG) private double verticalVelocityRps = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double horizontalVelocityRps = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double upwardVelocityRps = 0.0;

    public Indexer() {
        for (Stage stage : Stage.values()) {
            TalonFX motor = new TalonFX(stage.canId, RobotConstants.MECHANISM_BUS);
            PhoenixUtil.applyConfig(motor, stage.config, stage.name().toLowerCase() + " indexer");
            motors.put(stage, motor);
            velocityRequests.put(stage, new VelocityVoltage(0));
        }
        ParentDevice.optimizeBusUtilizationForAll(motors.values().toArray(new TalonFX[0]));
        for (TalonFX motor : motors.values()) {
            motor.getVelocity().setUpdateFrequency(10);
        }
    }

    @Override
    public void periodic() {
        verticalVelocityRps = getVelocityRps(Stage.VERTICAL);
        horizontalVelocityRps = getVelocityRps(Stage.HORIZONTAL);
        upwardVelocityRps = getVelocityRps(Stage.UPWARD);
    }

    public void run(Stage stage) {
        motors.get(stage).setControl(velocityRequests.get(stage).withVelocity(stage.forwardSpeedRps));
    }

    public void reverse(Stage stage) {
        motors.get(stage).setControl(velocityRequests.get(stage).withVelocity(stage.reverseSpeedRps));
    }

    public void stop(Stage stage) {
        motors.get(stage).setControl(stopRequest);
    }

    public double getVelocityRps(Stage stage) {
        return motors.get(stage).getVelocity().getValueAsDouble();
    }
}
