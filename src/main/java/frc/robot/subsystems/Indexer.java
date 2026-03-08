package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerType;

/**
 * Indexer subsystem with automatic telemetry via Epilogue.
 * 
 * <p>
 * All three TalonFX motors are automatically logged with health data.
 */
@Logged
public class Indexer extends SubsystemBase {
    @Logged
    private final TalonFX horizontalIndexerMotor;
    @Logged
    private final TalonFX verticalIndexerMotor;
    @Logged
    private final TalonFX upwardIndexerMotor;

    private final StatusSignal<Voltage> horizontalIndexerMotorVoltage;
    private final StatusSignal<Voltage> verticalIndexerMotorVoltage;
    private final StatusSignal<Voltage> upwardIndexerMotorVoltage;

    private final VelocityVoltage horizontalMotionController;
    private final VelocityVoltage verticalMotionController;
    private final VelocityVoltage upwardMotionController;

    public Indexer(CANBus canBus) {
        horizontalIndexerMotor = new TalonFX(IndexerConstants.horizontalIndexerMotorID, canBus);
        verticalIndexerMotor = new TalonFX(IndexerConstants.verticalIndexerMotorID, canBus);
        upwardIndexerMotor = new TalonFX(IndexerConstants.upwardIndexerMotorID, canBus);

        configureHorizontalIndexerMotor();
        configureVerticalIndexerMotor();
        configureUpwardIndexerMotor();

        horizontalMotionController = new VelocityVoltage(0);
        verticalMotionController = new VelocityVoltage(0);
        upwardMotionController = new VelocityVoltage(0);

        horizontalIndexerMotorVoltage = horizontalIndexerMotor.getMotorVoltage();
        verticalIndexerMotorVoltage = verticalIndexerMotor.getMotorVoltage();
        upwardIndexerMotorVoltage = upwardIndexerMotor.getMotorVoltage();

        horizontalIndexerMotorVoltage.setUpdateFrequency(50);
        verticalIndexerMotorVoltage.setUpdateFrequency(50);
        upwardIndexerMotorVoltage.setUpdateFrequency(50);
    }

    @Override
    public void periodic() {
        horizontalIndexerMotorVoltage.refresh();
        verticalIndexerMotorVoltage.refresh();
        upwardIndexerMotorVoltage.refresh();
    }

    private void configureHorizontalIndexerMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = horizontalIndexerMotor.getConfigurator().apply(IndexerConstants.HORIZONTAL_INDEXER_CONFIG, 0.1);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply horizontal indexer motor configs: " + status.toString());
        }
    }

    private void configureVerticalIndexerMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = verticalIndexerMotor.getConfigurator().apply(IndexerConstants.VERTICAL_INDEXER_CONFIG, 0.1);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply vertical indexer motor configs: " + status.toString());
        }
    }

    private void configureUpwardIndexerMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = upwardIndexerMotor.getConfigurator().apply(IndexerConstants.UPWARD_INDEXER_CONFIG, 0.1);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply upward indexer motor configs: " + status.toString());
        }
    }

    public void setIndexerSpeed(double speed, IndexerType indexer) {
        switch (indexer) {
            case HORIZONTAL:
                horizontalIndexerMotor.setControl(horizontalMotionController.withVelocity(speed));
                break;
            case VERTICAL:
                verticalIndexerMotor.setControl(verticalMotionController.withVelocity(speed));
                break;
            case UPWARD:
                upwardIndexerMotor.setControl(upwardMotionController.withVelocity(speed));
                break;
        }
    }

    public void stop() {
        horizontalIndexerMotor.stopMotor();
        verticalIndexerMotor.stopMotor();
        upwardIndexerMotor.stopMotor();
    }

    // ==================
    // MOTOR ACCESSORS (for FaultMonitor registration)
    // ==================

    public TalonFX getHorizontalMotor() {
        return horizontalIndexerMotor;
    }

    public TalonFX getVerticalMotor() {
        return verticalIndexerMotor;
    }

    public TalonFX getUpwardMotor() {
        return upwardIndexerMotor;
    }
}
