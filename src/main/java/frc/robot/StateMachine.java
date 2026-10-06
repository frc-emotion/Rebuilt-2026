package frc.robot;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.IndexerConstants.Stage;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.util.ControllerUtil;
import frc.robot.util.ShootingLookup;

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
    private final ShootingLookup shootingLookup;

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

    @Logged(importance = Logged.Importance.DEBUG) private Pose2d turretPose = new Pose2d();
    @Logged(importance = Logged.Importance.DEBUG) private Pose2d turretTargetPose = new Pose2d();

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

        shootingLookup = new ShootingLookup();
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
    }

    public void periodic() {
        readInputs();
        Resolution resolution = resolve(intakeRequested, shootRequested, clearRequested, intakeOut, atShooterSpeed);
        intakeState = resolution.intake();
        indexerState = resolution.indexer();
        shooterState = resolution.shooter();
        apply();
        updateTurretPoses();
    }

    private void readInputs() {
        vision.update();

        intakeOut = intake.isOut();
        atShooterSpeed = shootRequested && shooter.atSetpoint();
        isAligned = turret.atSetpoint() && hood.atSetpoint();

        // Computed every loop so lastYawDeg stays current while auto aim is active
        double currentYawDeg = getYawDeg();
        double yawDeltaRot = ControllerUtil.gyroCorrection(lastYawDeg, currentYawDeg);
        lastYawDeg = currentYawDeg;
        if (toggleManualTurret) {
            turretSetpointRot += ControllerUtil.turretJoystickUpdate(turretAxis.getAsDouble());
            turretSetpointRot += yawDeltaRot;
        } else {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                turretSetpointRot = vision.updateTurretSetpoint(alliance.get(), drivetrain.getState().Pose);
            }
        }

        if (toggleManualTurret){

            hoodSetpointRot += ControllerUtil.hoodJoystickUpdate(hoodAxis.getAsDouble());
        }
        else{
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                hoodSetpointRot = shootingLookup.getHoodAngle(alliance.get(), drivetrain.getState().Pose);
            }
        }
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
        if (intakeOut) {
            intake.runRoller();
        } else {
            intake.stopRoller();
        }        
        switch (indexerState) {
            case CLEARING -> {
                for (Stage stage : Stage.values()) indexer.reverse(stage);
            }
            case FEED_ALL -> {
                for (Stage stage : Stage.values()) indexer.run(stage);
            }
            case FEED_VERTICAL -> {
                indexer.run(Stage.VERTICAL);
                indexer.stop(Stage.HORIZONTAL);
                indexer.stop(Stage.UPWARD);
            }
            case STOPPED -> {
                for (Stage stage : Stage.values()) indexer.stop(stage);
            }
        }

        if (shooterState == ShooterState.IDLE) {
            shooter.coast();
        } else {
            if (toggleManualTurret){
                shooter.setVelocity(ShooterConstants.SHOOT_RPS);
            }
            else{
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    shooter.setVelocity(shootingLookup.getShooterSpeed(alliance.get(), drivetrain.getState().Pose));
                }                
            }
        }

        turretSetpointRot = turret.setSetpoint(turretSetpointRot);
        hoodSetpointRot = hood.setSetpoint(hoodSetpointRot);
    }

    /** Field-relative turret arrows for AdvantageScope: where the turret points and where it is told to point. */
    private void updateTurretPoses() {
        Pose2d robotPose = drivetrain.getState().Pose;
        turretPose = Vision.turretFieldPose(robotPose, turret.getPositionRot());
        turretTargetPose = Vision.turretFieldPose(robotPose, turretSetpointRot);
    }

    private double getYawDeg() {
        return drivetrain.getPigeon2().getYaw().getValueAsDouble();
    }
}