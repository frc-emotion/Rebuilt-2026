package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.util.TurretAimingCalculator;

/**
 * Turret command: always drives to a position setpoint via MotionMagic.
 *
 * TRACKING (default): turret holds last setpoint. Vision frames update it.
 * MANUAL (left stick held): operator joystick controls turret via voltage.
 *
 * There is no timeout. The turret always tries to reach its setpoint.
 * Vision just updates the setpoint when a fresh tag is seen.
 */
@Logged
public class TurretAutoAimCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final Turret turret;
    private final TurretAimingCalculator calculator;
    private final DoubleSupplier joystickSupplier;
    private final BooleanSupplier manualOverride;
    private final BooleanSupplier isPassing;

    @Logged(importance = Logged.Importance.CRITICAL) private String state = "TRACKING";
    @Logged(importance = Logged.Importance.CRITICAL) private double distanceToHubMeters = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double visionTxDeg = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean visionActive = false;
    @Logged(importance = Logged.Importance.CRITICAL) private int trackedTagId = -1;
    @Logged(importance = Logged.Importance.CRITICAL) public double targetPositionRot = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double currentPositionRot = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double turretErrorRot = 0.0;
    @Logged(importance = Logged.Importance.DEBUG) private double gyroFeedforwardRot = 0.0;

    @Logged(importance = Logged.Importance.CRITICAL) private double lastGyroYawDeg = 0.0;

    @Logged(importance = Logged.Importance.CRITICAL) private double omega = 0.0;

    private static final double DEADBAND_DEG = 3.0;
    private static final double MANUAL_DEADBAND = 0.08;
    private static final boolean GYRO_FF_ENABLED = true;

    private double lastVisionTimestamp = 0.0;
    private boolean freshVisionThisCycle = false;
    private boolean wasPassing = false;
    private boolean passingDirectionLocked = false;


    public TurretAutoAimCommand(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            Turret turret,
            DoubleSupplier joystickSupplier,
            BooleanSupplier manualOverride,
            BooleanSupplier isPassing) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.turret = turret;
        this.calculator = new TurretAimingCalculator();
        this.joystickSupplier = joystickSupplier;
        this.manualOverride = manualOverride;
        this.isPassing = isPassing;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        calculator.clearAllianceCache();
        targetPositionRot = turret.getTurretMotor().getPosition().getValueAsDouble();
        lastVisionTimestamp = 0.0;
        visionTxDeg = 0.0;
        freshVisionThisCycle = false;
        wasPassing = false;
        passingDirectionLocked = false;
        lastGyroYawDeg = drivetrain != null ? drivetrain.getPigeon2().getYaw().getValueAsDouble() : 0.0;
    }

    @Override
    public void execute() {
        boolean manual = manualOverride.getAsBoolean();
        boolean passing = isPassing.getAsBoolean();

        state = manual ? "MANUAL" : "TRACKING";
        
        double turretPos = turret.getTurretMotor().getPosition().getValueAsDouble();

        if (manual) {
            double input = MathUtil.applyDeadband(joystickSupplier.getAsDouble(), MANUAL_DEADBAND);
            turret.setTurretVoltage(MathUtil.clamp(input, -1, 1));
            targetPositionRot = turretPos;
        } else if (passing) {
            state = "PASSING";
            currentPositionRot = turretPos;
            applyGyroFF();
            readPassing();
            if (freshVisionThisCycle) {
                targetPositionRot = currentPositionRot + visionTxDeg / 360.0;
                passingDirectionLocked = true;
            }
            targetPositionRot = turret.moveTurret(Rotations.of(targetPositionRot)).in(Rotations);
            }
            
         else {
            readVision();
            ChassisSpeeds speeds = drivetrain.getState().Speeds;
            omega = speeds.omegaRadiansPerSecond;

            currentPositionRot = turretPos;
            applyGyroFF();
            if (freshVisionThisCycle) {
                targetPositionRot = currentPositionRot + visionTxDeg / 360.0;
            }
            applyOmega();
            targetPositionRot = turret.moveTurret(Rotations.of(targetPositionRot)).in(Rotations);
        }

        wasPassing = passing;
        turretErrorRot = targetPositionRot - turretPos;
    }

    private void applyOmega() {
        return;
        // targetPositionRot += (omega * TurretConstants.omegaFeedforwardMultiplier);

    }

    private void readVision() {
        freshVisionThisCycle = false;
        visionActive = false;
        if (vision == null || !vision.isTurretResultFresh()) return;

        double ts = vision.getTurretResultTimestamp();
        if (ts == lastVisionTimestamp) return;
        lastVisionTimestamp = ts;

        if (!vision.isSeeingHubTag()) return;

        visionTxDeg = vision.getYawToHubDeg();
        distanceToHubMeters = vision.getDistanceToHub();
        trackedTagId = vision.getTrackedTagId();
        visionActive = true;
        freshVisionThisCycle = true;
    }

    public void readPassing() {
            freshVisionThisCycle = false;
            visionActive = false; 
            if (vision == null || !vision.isTurretResultFresh()) return;
            double ts  = vision.getTurretResultTimestamp();
            if (ts ==lastVisionTimestamp) return;
            lastVisionTimestamp = ts;

            if (!vision.isSeeingPassingTag()) return;

            visionTxDeg = vision.getYawToPassingTagDeg();
            distanceToHubMeters = vision.getDistanceToPassingTag();
            trackedTagId = vision.getTrackedPassingTagId();
            visionActive = true;
            freshVisionThisCycle = true;
    }

    private void applyGyroFF() {
        gyroFeedforwardRot = 0.0;
        if (!GYRO_FF_ENABLED || drivetrain == null) return;

        double currentYawDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        double deltaDeg = currentYawDeg - lastGyroYawDeg;
        lastGyroYawDeg = currentYawDeg;

        gyroFeedforwardRot = deltaDeg / 360.0;
        targetPositionRot += gyroFeedforwardRot;
    }

    // private void applyGyroFF() {
    //     gyroFeedforwardRot = 0.0;
    //     if (!GYRO_FF_ENABLED || drivetrain == null) return;
    //     double gyroRate = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
    //     gyroFeedforwardRot = (gyroRate / 360.0) * LOOP_PERIOD_SEC*1.75;
       
    //     targetPositionRot += gyroFeedforwardRot;
    // }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean isAimed() {
        return !manualOverride.getAsBoolean() && Math.abs(visionTxDeg) < DEADBAND_DEG;
    }

    public boolean isTracking() {
        return !manualOverride.getAsBoolean();
    }

    public double getDistanceToHub() {
        return distanceToHubMeters;
    }

    public TurretAimingCalculator getCalculator() {
        return calculator;
    }

    public boolean isPassingLocked() {
        return passingDirectionLocked;
    }

    public boolean currentlyPassing() {
        return isPassing.getAsBoolean();
    }

    
}
