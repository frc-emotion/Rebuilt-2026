package frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class IntakeConstants { 
    public static final int intakeMotorID = 20; 
    public static final int rollerMotorID = 21;
    public static final double IntakeCurrentSpike = 20;

    public static final Angle INTAKE_IN_ANGLE = Degrees.of(-8);

    // raw rotations: intake = 0.34 so we gotta multiply by 360

    public static final Angle INTAKE_OUT_ANGLE = Degrees.of(126.7);

    public static final Angle TOLERANCE = Degrees.of(5);


    public static final TalonFXConfiguration INTAKE_CONFIG = new TalonFXConfiguration();

    static{
        INTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        INTAKE_CONFIG.Slot0.kG = 0;
        INTAKE_CONFIG.Slot0.kS = 0;
        INTAKE_CONFIG.Slot0.kV = 0.0;
        INTAKE_CONFIG.Slot0.kA = 0.0;
        INTAKE_CONFIG.Slot0.kP = 13;
        INTAKE_CONFIG.Slot0.kI = 0.0;
        INTAKE_CONFIG.Slot0.kD = 0.0;

        // MotionMagic constraints — prevents slapdown from slamming. TODO: tune on robot.
        INTAKE_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 2.0; // RPS
        INTAKE_CONFIG.MotionMagic.MotionMagicAcceleration = 4.0;   // RPS^2
        INTAKE_CONFIG.MotionMagic.MotionMagicJerk = 40.0;          // Smoothing

        INTAKE_CONFIG.Feedback.SensorToMechanismRatio = 27.0;
        //27:1



    }

    public static final double INTAKE_ROLLER_VELOCITY = 45;

    public static final TalonFXConfiguration ROLLER_CONFIG = new TalonFXConfiguration();

    static{
        ROLLER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        ROLLER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        ROLLER_CONFIG.Slot0.kG = 0;
        ROLLER_CONFIG.Slot0.kS = 0;
        ROLLER_CONFIG.Slot0.kV = 0;
        ROLLER_CONFIG.Slot0.kA = 0.0;
        ROLLER_CONFIG.Slot0.kP = 0.4;
        ROLLER_CONFIG.Slot0.kI = 0.0;
        ROLLER_CONFIG.Slot0.kD = 0.00;


    }

   
   

}
