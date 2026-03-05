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
    // @Logged
    // private final TalonFX verticalIndexerMotor; // DISABLED: not installed
    @Logged
    private final TalonFX upwardIndexerMotor;

    private final StatusSignal<Voltage> horizontalIndexerMotorVoltage;
    // private final StatusSignal<Voltage> verticalIndexerMotorVoltage; // DISABLED
    private final StatusSignal<Voltage> upwardIndexerMotorVoltage;

    private final VelocityVoltage horizontalMotionController;
    // private final VelocityVoltage verticalMotionController; // DISABLED
    private final VelocityVoltage upwardMotionController;

    public Indexer(CANBus canBus) {
        horizontalIndexerMotor = new TalonFX(IndexerConstants.horizontalIndexerMotorID, canBus);
        // verticalIndexerMotor = new TalonFX(IndexerConstants.verticalIndexerMotorID, canBus); // DISABLED
        upwardIndexerMotor = new TalonFX(IndexerConstants.upwardIndexerMotorID, canBus);

        configureHorizontalIndexerMotor();
        // configureVerticalIndexerMotor(); // DISABLED
        configureUpwardIndexerMotor();

        horizontalMotionController = new VelocityVoltage(0);
        // verticalMotionController = new VelocityVoltage(0); // DISABLED
        upwardMotionController = new VelocityVoltage(0);

        horizontalIndexerMotorVoltage = horizontalIndexerMotor.getMotorVoltage();
        // verticalIndexerMotorVoltage = verticalIndexerMotor.getMotorVoltage(); // DISABLED
        upwardIndexerMotorVoltage = upwardIndexerMotor.getMotorVoltage();

        horizontalIndexerMotorVoltage.setUpdateFrequency(50);
        // verticalIndexerMotorVoltage.setUpdateFrequency(50); // DISABLED
        upwardIndexerMotorVoltage.setUpdateFrequency(50);
    }

    @Override
    public void periodic() {
        horizontalIndexerMotorVoltage.refresh();
        // verticalIndexerMotorVoltage.refresh(); // DISABLED
        upwardIndexerMotorVoltage.refresh();
    }

    private void configureHorizontalIndexerMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = horizontalIndexerMotor.getConfigurator().apply(IndexerConstants.HORIZONTAL_INDEXER_CONFIG, 0.1);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.err.println("Could not apply horizontal indexer motor configs: " + status.toString());
        }
    }

    // private void configureVerticalIndexerMotor() { // DISABLED: not installed
    //     StatusCode status = StatusCode.StatusCodeNotInitialized;
    //     for (int i = 0; i < 5; ++i) {
    //         status = verticalIndexerMotor.getConfigurator().apply(IndexerConstants.VERTICAL_INDEXER_CONFIG, 0.1);
    //         if (status.isOK()) break;
    //     }
    //     if (!status.isOK()) {
    //         System.err.println("Could not apply vertical indexer motor configs: " + status.toString());
    //     }
    // }

    private void configureUpwardIndexerMotor() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = upwardIndexerMotor.getConfigurator().apply(IndexerConstants.UPWARD_INDEXER_CONFIG, 0.1);
            if (status.isOK()) break;
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
            // case VERTICAL: // DISABLED: not installed
            //     verticalIndexerMotor.setControl(verticalMotionController.withVelocity(speed));
            //     break;
            case VERTICAL: break; // DISABLED: motor not installed
            case UPWARD:
                upwardIndexerMotor.setControl(upwardMotionController.withVelocity(speed));
                break;
        }
    }

    public void stop() {
        horizontalIndexerMotor.stopMotor();
        // verticalIndexerMotor.stopMotor(); // DISABLED
        upwardIndexerMotor.stopMotor();
    }

    // ==================
    // MOTOR ACCESSORS (for FaultMonitor registration)
    // ==================

    public TalonFX getHorizontalMotor() {
        return horizontalIndexerMotor;
    }

    // public TalonFX getVerticalMotor() { // DISABLED: not installed
    //     return verticalIndexerMotor;
    // }

    public TalonFX getUpwardMotor() {
        return upwardIndexerMotor;
    }
}
