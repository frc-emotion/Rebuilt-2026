package frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class IntakeConstants { 
    public static final int intakeMotorID = 20; 
    public static final int rollerMotorID = 21;
    public static final double IntakeCurrentSpike = 20;

    public static final Angle INTAKE_IN_ANGLE = Degrees.of(0);
    public static final Angle INTAKE_OUT_ANGLE = Degrees.of(85);

    public static final Angle TOLERANCE = Degrees.of(0.5);


    public static final TalonFXConfiguration INTAKE_CONFIG = new TalonFXConfiguration();

    static{
        INTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        INTAKE_CONFIG.Slot0.kG = 0.53; // Volts to overcome gravity
        INTAKE_CONFIG.Slot0.kS = 0.5; // Volts to overcome static friction
        INTAKE_CONFIG.Slot0.kV = 0.0; // Volts for a velocity target of 1 rps
        INTAKE_CONFIG.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/s
        INTAKE_CONFIG.Slot0.kP = 25;
        INTAKE_CONFIG.Slot0.kI = 0.0;
        INTAKE_CONFIG.Slot0.kD = 0.00;
    }

    public static final double INTAKE_ROLLER_VELOCITY = 40;

    public static final TalonFXConfiguration ROLLER_CONFIG = new TalonFXConfiguration();

    static{
        ROLLER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        ROLLER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        ROLLER_CONFIG.Slot0.kG = 0; // Volts to overcome gravity
        ROLLER_CONFIG.Slot0.kS = 0; // Volts to overcome static friction
        ROLLER_CONFIG.Slot0.kV = 0.0; // Volts for a velocity target of 1 rps
        ROLLER_CONFIG.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/s
        ROLLER_CONFIG.Slot0.kP = 25;
        ROLLER_CONFIG.Slot0.kI = 0.0;
        ROLLER_CONFIG.Slot0.kD = 0.00;


    }

   
   

}
