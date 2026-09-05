package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

public final class PhoenixUtil {
    private static final int CONFIG_ATTEMPTS = 5;
    private static final double CONFIG_TIMEOUT_SECONDS = 0.1;

    private PhoenixUtil() {}

    public static void applyConfig(TalonFX motor, TalonFXConfiguration config, String deviceName) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_ATTEMPTS && !status.isOK(); i++) {
            status = motor.getConfigurator().apply(config, CONFIG_TIMEOUT_SECONDS);
        }
        report(status, deviceName);
    }

    public static void applyConfig(CANcoder encoder, CANcoderConfiguration config, String deviceName) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_ATTEMPTS && !status.isOK(); i++) {
            status = encoder.getConfigurator().apply(config, CONFIG_TIMEOUT_SECONDS);
        }
        report(status, deviceName);
    }

    private static void report(StatusCode status, String deviceName) {
        if (!status.isOK()) {
            DriverStation.reportError("Could not apply " + deviceName + " config: " + status, false);
        }
    }
}
