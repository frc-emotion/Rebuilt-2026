package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
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
 * Full-vision turret auto-aim (mode 3: FULL_VISION).
 *
 * <p>Layered architecture (254/6328-proven):
 * <ol>
 *   <li><b>Odometry base</b> — atan2(hub − robot) minus heading → turret setpoint</li>
 *   <li><b>Vision correction</b> — additive P-controller on turret camera tx error</li>
 *   <li><b>Gyro feedforward</b> — counters robot yaw rate lag</li>
 *   <li><b>Hood + flywheel</b> — distance-based dual interpolation tables</li>
 * </ol>
 *
 * <p>The drivetrain's pose estimator fuses wheel odometry with vision measurements
 * (fed by Robot.java calling updateVisionPoseEstimates). The turret camera provides
 * an additional fine-grained correction on top of that fused pose.
 *
 * <p>Vision is required for this mode. If vision is null, falls back to pure odometry.
 */
@Logged
public class TurretAutoAimCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final TurretAimingCalculator calculator;

    @Logged private double distanceToHubMeters = 0.0;
    @Logged private double encoderSetpointRot = TurretConstants.TURRET_FORWARD_POSITION;
    @Logged private double visionCorrectionDeg = 0.0;
    @Logged private double gyroFFVolts = 0.0;
    @Logged private boolean visionActive = false;
    @Logged private double hoodSetpointRot = 0.0;
    @Logged private double shooterSetpointRPS = 0.0;

    private static final double kVisionP = 0.003;
    private static final double kGyroFF = 0.15;

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
        encoderSetpointRot = turret.getTurretMotor().getPosition().getValueAsDouble();
        lastVisionTimestamp = 0.0;
    }

    @Override
    public void execute() {
        // === Layer 1: Odometry-based setpoint (always runs) ===
        Pose2d robotPose = drivetrain.getState().Pose;
        AimingParameters params = calculator.calculate(robotPose);
        distanceToHubMeters = params.distanceToHub();

        double odomSetpoint = TurretConstants.TURRET_FORWARD_POSITION
                + params.turretAngle().getRotations()
                + TurretConstants.TURRET_AIM_OFFSET;

        // === Layer 2: Vision correction (additive, only on fresh frames) ===
        visionActive = false;
        visionCorrectionDeg = 0.0;
        double visionOffsetRot = 0.0;

        if (vision != null
                && vision.isTurretResultFresh()
                && vision.turretCameraHasTargets()) {
            double currentTimestamp = vision.getTurretResultTimestamp();
            if (currentTimestamp != lastVisionTimestamp) {
                lastVisionTimestamp = currentTimestamp;

                var target = vision.getTurretCameraBestTarget();
                if (target.isPresent()
                        && (VisionConstants.BENCH_TEST_ANY_TAG
                            || VisionConstants.isHubTag(target.get().getFiducialId()))) {
                    double tx = target.get().getYaw();
                    visionCorrectionDeg = tx;
                    visionActive = true;
                    visionOffsetRot = kVisionP * tx;
                }
            }
        }

        encoderSetpointRot = odomSetpoint + visionOffsetRot;

        // === Layer 3: Gyro feedforward ===
        double robotYawRPS = drivetrain.getPigeon2()
                .getAngularVelocityZWorld().getValueAsDouble() / 360.0;
        gyroFFVolts = kGyroFF * (-robotYawRPS);

        turret.moveTurretWithWrap(Rotations.of(encoderSetpointRot), gyroFFVolts);

        // === Hood + Flywheel from interpolation tables ===
        hoodSetpointRot = params.hoodAngle().getRotations();
        shooterSetpointRPS = params.flywheelRPS();
        hood.setHoodAngle(Rotations.of(hoodSetpointRot));
        shooter.setShooterSpeed(RotationsPerSecond.of(shooterSetpointRPS));
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        hood.stop();
        shooter.stop();
    }

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
