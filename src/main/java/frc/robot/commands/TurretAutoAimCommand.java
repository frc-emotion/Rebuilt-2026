package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.util.TurretAimingCalculator;
import frc.robot.util.TurretAimingCalculator.AimingParameters;

/**
 * Layered turret auto-aim: always tracks the alliance hub.
 *
 * <p>Architecture (254/6328-proven):
 * <ol>
 *   <li><b>Pose-based</b> (always) — atan2 to hub minus robot heading</li>
 *   <li><b>Vision correction</b> (additive, when turret camera sees hub tag)</li>
 *   <li><b>Gyro feedforward</b> — counters robot rotation lag</li>
 *   <li><b>Barrier clamp</b> — encoder setpoint stays in safe range</li>
 *   <li><b>Hood + flywheel</b> — distance-based interpolation tables</li>
 * </ol>
 *
 * <p>Reads drivetrain pose and pigeon gyro. Does NOT command drivetrain movement.
 * Vision is optional — pass null if cameras are not mounted yet.
 */
@Logged
public class TurretAutoAimCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision; // nullable
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final TurretAimingCalculator calculator;

    @Logged private double distanceToHubMeters = 0.0;
    @Logged private double encoderSetpointRot = TurretConstants.TURRET_FORWARD_POSITION;
    @Logged private double visionCorrectionDeg = 0.0;
    @Logged private double gyroFFVolts = 0.0;
    @Logged private boolean visionActive = false;

    // rotations per degree of camera tx error
    private static final double kVisionP = 0.003;
    // volts per RPS of robot yaw rate
    private static final double kGyroFF = 0.15;

    // Tracks the last vision timestamp so we only update setpoint on FRESH frames
    private double lastVisionTimestamp = 0.0;

    public TurretAutoAimCommand(
            CommandSwerveDrivetrain drivetrain,
            Vision vision,
            Turret turret,
            Hood hood,
            Shooter shooter) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.calculator = new TurretAimingCalculator();
        addRequirements(turret, hood, shooter);
    }

    @Override
    public void initialize() {
        calculator.clearAllianceCache();
        // Seed the setpoint to wherever the turret currently is, so it doesn't
        // jump to TURRET_FORWARD_POSITION on startup.
        encoderSetpointRot = turret.getTurretMotor().getPosition().getValueAsDouble();
        lastVisionTimestamp = 0.0;
    }

    @Override
    public void execute() {
        // ============================================================
        // VISION-ONLY MODE (odometry disabled for bench testing)
        // Turret tracks hub tag yaw directly via P-controller.
        // Re-enable odometry layers when field pose is reliable.
        // ============================================================

        visionActive = false;
        visionCorrectionDeg = 0.0;
        gyroFFVolts = 0.0;

        // Only recompute the setpoint when the turret camera delivers a FRESH frame.
        // This prevents the old bug where stale tx was re-applied every robot cycle,
        // turning the P-controller into an integrator that drifts in one direction.
        if (vision != null
                && vision.isTurretResultFresh()
                && vision.turretCameraHasTargets()) {
            double currentTimestamp = vision.getTurretResultTimestamp();
            if (currentTimestamp != lastVisionTimestamp) {
                lastVisionTimestamp = currentTimestamp;

                var target = vision.getTurretCameraBestTarget();
                if (target.isPresent()
                        && (VisionConstants.BENCH_TEST_ANY_TAG || VisionConstants.isHubTag(target.get().getFiducialId()))) {
                    double tx = target.get().getYaw(); // degrees off camera center
                    visionCorrectionDeg = tx;
                    visionActive = true;

                    // P-controller: current position + error converted to rotations.
                    // Computed ONCE per fresh vision frame, then MotionMagic holds
                    // the setpoint until the next frame arrives.
                    // Flip sign below if turret tracks the wrong direction.
                    double currentPos = turret.getTurretMotor().getPosition().getValueAsDouble();
                    encoderSetpointRot = currentPos + (kVisionP * tx);
                }
            }
        }

        // ALWAYS command the turret — holds last setpoint when no fresh vision.
        // This replaces the old "do nothing" fallback that left the motor in an
        // undefined state.
        turret.moveTurret(Rotations.of(encoderSetpointRot), gyroFFVolts);

        // --- ODOMETRY DISABLED ---
        // Pose2d robotPose = drivetrain.getState().Pose;
        // AimingParameters params = calculator.calculate(robotPose);
        // distanceToHubMeters = params.distanceToHub();
        // double setpoint = TurretConstants.TURRET_FORWARD_POSITION
        //         + params.turretAngle().getRotations();
        // double robotYawRPS = drivetrain.getPigeon2()
        //         .getAngularVelocityZWorld().getValueAsDouble() / 360.0;
        // gyroFFVolts = kGyroFF * (-robotYawRPS);
        // hood.setHoodAngle(Rotations.of(params.hoodAngle().getRotations()));
        // shooter.setShooterSpeed(RotationsPerSecond.of(params.flywheelRPM() / 60.0));
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean isAimed() {
        double errorRot = Math.abs(
                turret.getTurretMotor().getPosition().getValueAsDouble() - encoderSetpointRot);
        return errorRot < TurretConstants.turretTolerance;
    }

    public double getDistanceToHub() {
        return distanceToHubMeters;
    }
}
