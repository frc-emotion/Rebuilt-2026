package frc.robot;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.IndexerConstants.Stage;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * Sole writer to the mechanism subsystems. Each loop: read requests and conditions,
 * resolve one state per mechanism, then re-send every output. Nothing here remembers
 * what it sent last loop, so releasing a button needs no cleanup logic.
 */
@Logged
public class StateMachine {
    public enum IntakeState { STOWED, DEPLOYING, DEPLOYED }
    public enum IndexerState { STOPPED, FEED_VERTICAL, FEED_ALL, CLEARING }
    public enum ShooterState { IDLE, SPINNING_UP, AT_SPEED }

    /** Pure decision output. Kept separate from the hardware so it can be unit tested. */
    public record Resolution(IntakeState intake, IndexerState indexer, ShooterState shooter) {}

    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;
    private final Indexer indexer;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final Vision vision;
    private final DoubleSupplier turretAxis;
    private final DoubleSupplier hoodAxis;

    // Initialized so nothing is null before the first enable
    @NotLogged private Optional<Alliance> allianceColor = Optional.empty();
    private Pose2d currentPose = new Pose2d();

    @Logged(importance  = Logged.Importance.CRITICAL) private boolean toggleManualTurret = false;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean intakeRequested = false;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean shootRequested = false;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean clearRequested = false;
    @Logged(importance = Logged.Importance.CRITICAL) private double turretSetpointRot = 0.0;
    @Logged(importance = Logged.Importance.CRITICAL) private double hoodSetpointRot = 0.0;

    @Logged(importance = Logged.Importance.CRITICAL) private boolean atShooterSpeed = false;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean intakeOut = false;
    @Logged(importance = Logged.Importance.CRITICAL) private boolean isAligned = false;

    @Logged(importance = Logged.Importance.CRITICAL) private IntakeState intakeState = IntakeState.STOWED;
    @Logged(importance = Logged.Importance.CRITICAL) private IndexerState indexerState = IndexerState.STOPPED;
    @Logged(importance = Logged.Importance.CRITICAL) private ShooterState shooterState = ShooterState.IDLE;

    @Logged(importance = Logged.Importance.DEBUG) private double gyroCorrectionRot = 0.0;
    private double lastYawDeg = 0.0;

    public StateMachine(CommandSwerveDrivetrain drivetrain, Intake intake, Indexer indexer,
            Turret turret, Hood hood, Shooter shooter, Vision vision,
            DoubleSupplier turretAxis, DoubleSupplier hoodAxis) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.indexer = indexer;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.vision = vision;
        this.turretAxis = turretAxis;
        this.hoodAxis = hoodAxis;
    }

    // Requests
    public void toggleManualTurret(){toggleManualTurret = !toggleManualTurret;}
    public void setIntake(boolean out) { intakeRequested = out; }
    public void toggleIntake() { intakeRequested = !intakeRequested; }
    public void setShoot(boolean shoot) { shootRequested = shoot; }
    public void setClear(boolean clear) { clearRequested = clear; }
    public void setTurretSetpoint(double rot) { turretSetpointRot = rot; }
    public void setHoodSetpoint(double rot) { hoodSetpointRot = rot; }

    /** Call on every enable so nothing jumps if a mechanism was moved by hand while disabled. */
    public void onEnable() {
        intakeRequested = false;
        shootRequested = false;
        clearRequested = false;
        turretSetpointRot = turret.getPositionRot();
        hoodSetpointRot = hood.getPositionRot();
        lastYawDeg = getYawDeg();
        allianceColor = DriverStation.getAlliance();
    }

    public void periodic() {
        readInputs();
        Resolution resolution = resolve(intakeRequested, shootRequested, clearRequested, intakeOut, atShooterSpeed);
        intakeState = resolution.intake();
        indexerState = resolution.indexer();
        shooterState = resolution.shooter();
        apply();
    }

    private void readInputs() {
        vision.update();

        // Refresh alliance (can arrive late from the FMS) and pose every loop,
        // and push them to vision BEFORE asking it for an aim angle
        allianceColor = DriverStation.getAlliance();
        currentPose = drivetrain.getState().Pose;
        vision.setAllianceColor(allianceColor);
        vision.setCurrentPose(currentPose);

        intakeOut = intake.isOut();
        atShooterSpeed = shootRequested && shooter.atSetpoint();
        isAligned = turret.atSetpoint() && hood.atSetpoint();

        if (toggleManualTurret) {
            turretSetpointRot += shapeStick(turretAxis.getAsDouble())
                * OperatorConstants.TURRET_JOYSTICK_RATE_ROT_PER_SEC * RobotConstants.LOOP_PERIOD_SECONDS;
            turretSetpointRot += gyroCorrection();
        } else {
            // Auto aim: turret target comes from vision
            Optional<Angle> turretToHub = vision.getTurretToHub();
            if (turretToHub.isPresent()) {
                double targetRot = turretToHub.get().in(Rotations);
                if (Math.abs(targetRot - turret.getPositionRot()) > VisionConstants.turretDiffTolerance) {
                    turretSetpointRot = targetRot + 0.25;
                }
            }
        }

        hoodSetpointRot += shapeStick(hoodAxis.getAsDouble())
                * OperatorConstants.HOOD_JOYSTICK_RATE_ROT_PER_SEC * RobotConstants.LOOP_PERIOD_SECONDS;
    }

    public static Resolution resolve(boolean intakeRequested, boolean shootRequested, boolean clearRequested,
            boolean intakeOut, boolean atShooterSpeed) {
        IntakeState intakeState = !intakeRequested ? IntakeState.STOWED
                : intakeOut ? IntakeState.DEPLOYED
                : IntakeState.DEPLOYING;

        IndexerState indexerState = clearRequested ? IndexerState.CLEARING
                : (shootRequested && atShooterSpeed) ? IndexerState.FEED_ALL
                : (intakeOut || shootRequested) ? IndexerState.FEED_VERTICAL
                : IndexerState.STOPPED;

        ShooterState shooterState = !shootRequested ? ShooterState.IDLE
                : atShooterSpeed ? ShooterState.AT_SPEED
                : ShooterState.SPINNING_UP;

        return new Resolution(intakeState, indexerState, shooterState);
    }

    private void apply() {
        intake.setDeployed(intakeRequested);
        // if (intakeOut) {
        //     intake.runRoller();
        // } else {
        //     intake.stopRoller();
        // }        
        // switch (indexerState) {
        //     case CLEARING -> {
        //         for (Stage stage : Stage.values()) indexer.reverse(stage);
        //     }
        //     case FEED_ALL -> {
        //         for (Stage stage : Stage.values()) indexer.run(stage);
        //     }
        //     case FEED_VERTICAL -> {
        //         indexer.run(Stage.VERTICAL);
        //         indexer.stop(Stage.HORIZONTAL);
        //         indexer.stop(Stage.UPWARD);
        //     }
        //     case STOPPED -> {
        //         for (Stage stage : Stage.values()) indexer.stop(stage);
        //     }
        // }

        indexer.stop(Stage.VERTICAL);
        indexer.stop(Stage.HORIZONTAL);
        indexer.stop(Stage.UPWARD);

        if (shooterState == ShooterState.IDLE) {
            shooter.coast();
        } else {
            shooter.setVelocity(ShooterConstants.SHOOT_RPS);
        }

        turretSetpointRot = turret.setSetpoint(turretSetpointRot);
        hoodSetpointRot = hood.setSetpoint(hoodSetpointRot);
    }

    /** Deadband then a power curve, so small deflections give fine adjustment and full deflection gives full rate. */
    static double shapeStick(double raw) {
        double input = MathUtil.applyDeadband(raw, OperatorConstants.STICK_DEADBAND);
        return Math.copySign(Math.pow(Math.abs(input), OperatorConstants.STICK_CURVE_EXPONENT), input);
    }

    /** Robot yaw change since last loop, in turret rotations, so the turret holds a field heading. */
    private double gyroCorrection() {
        double yawDeg = getYawDeg();
        double deltaDeg = yawDeg - lastYawDeg;
        lastYawDeg = yawDeg;
        gyroCorrectionRot = deltaDeg / 360.0 * OperatorConstants.TURRET_GYRO_CORRECTION_GAIN;
        return gyroCorrectionRot;
    }

    private double getYawDeg() {
        return drivetrain.getPigeon2().getYaw().getValueAsDouble();
    }

    public Pose2d getPose2d(){
        return currentPose;
    }
}