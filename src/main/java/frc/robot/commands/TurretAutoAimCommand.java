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
    @Logged private double encoderSetpointRot = 0.0;
    @Logged private double visionCorrectionDeg = 0.0;
    @Logged private double gyroFFVolts = 0.0;
    @Logged private boolean visionActive = false;

    // rotations per degree of camera tx error
    private static final double kVisionP = 0.003;
    // volts per RPS of robot yaw rate
    private static final double kGyroFF = 0.15;

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

        if (vision != null && vision.turretCameraHasTargets()) {
            var target = vision.getTurretCameraBestTarget();
            if (target.isPresent() && VisionConstants.isHubTag(target.get().getFiducialId())) {
                double tx = target.get().getYaw(); // degrees off camera center
                visionCorrectionDeg = tx;
                visionActive = true;

                // P-controller: current position + error converted to rotations.
                // If tag is right of center (positive yaw), turret rotates to follow.
                // Flip sign below if turret tracks the wrong direction.
                double currentPos = turret.getTurretMotor().getPosition().getValueAsDouble();
                double setpoint = currentPos + (tx / 360.0);

                setpoint = MathUtil.clamp(setpoint,
                        TurretConstants.TURRET_REVERSE_LIMIT,
                        TurretConstants.TURRET_FORWARD_LIMIT);
                encoderSetpointRot = setpoint;

                turret.moveTurret(Rotations.of(setpoint), 0);
            }
        }
        // If no hub tag visible, turret holds last position (no new command sent).

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
