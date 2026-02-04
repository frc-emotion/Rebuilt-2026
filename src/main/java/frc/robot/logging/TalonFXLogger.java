package frc.robot.logging;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/**
 * Custom Epilogue logger for TalonFX motors.
 * 
 * <p>
 * This logger automatically extracts critical motor health data for any TalonFX
 * field annotated with {@code @Logged}. Data logged includes:
 * <ul>
 * <li>Supply current (amps) - current drawn from battery, critical for brownout
 * diagnosis</li>
 * <li>Stator current (amps) - current through motor windings</li>
 * <li>Motor voltage (volts) - voltage applied to motor</li>
 * <li>Temperature (°C) - motor temperature for overheat detection</li>
 * <li>Position (rotations) - rotor position</li>
 * <li>Velocity (rotations/sec) - rotor velocity</li>
 * <li>Fault flags - hardware/firmware/boot/undervoltage faults</li>
 * </ul>
 * 
 * <p>
 * Usage: Simply add {@code @Logged} to any TalonFX field in your subsystem.
 * This logger is automatically discovered and used by Epilogue.
 * 
 * <pre>
 * {
 *     &#64;code
 *     &#64;Logged
 *     public class Intake extends SubsystemBase {
 *         @Logged
 *         private final TalonFX intakeMotor = new TalonFX(1);
 *     }
 * }
 * </pre>
 * 
 * @see <a href=
 *      "https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html">WPILib
 *      Epilogue Documentation</a>
 */
@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {

    public TalonFXLogger() {
        super(TalonFX.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFX motor) {
        // Current monitoring - critical for brownout diagnosis
        backend.log("SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        backend.log("StatorCurrent", motor.getStatorCurrent().getValueAsDouble());

        // Voltage monitoring
        backend.log("MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
        backend.log("SupplyVoltage", motor.getSupplyVoltage().getValueAsDouble());

        // Temperature monitoring - detect overheating
        backend.log("Temperature", motor.getDeviceTemp().getValueAsDouble());

        // Motion data
        backend.log("Position", motor.getPosition().getValueAsDouble());
        backend.log("Velocity", motor.getVelocity().getValueAsDouble());

        // Fault monitoring - log only if faults present (reduces noise)
        if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
            backend.log("FaultHardware", motor.getFault_Hardware().getValue());
            backend.log("FaultBootDuringEnable", motor.getFault_BootDuringEnable().getValue());
            backend.log("FaultUnderVoltage", motor.getFault_Undervoltage().getValue());
            backend.log("FaultUnlicensedFeature", motor.getFault_UnlicensedFeatureInUse().getValue());
        }
    }
}
