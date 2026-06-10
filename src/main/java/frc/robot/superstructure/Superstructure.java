package frc.robot.superstructure;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIO.Stage;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ShotCalculator;
import java.util.Optional;
import java.util.function.BooleanSupplier;

/**
 * The robot-level state machine. Callers request GOALS (outcomes); this class plans the path
 * through states (Transitions, pure + tested) and commands the mechanism setpoints each loop.
 * Mechanisms stay dumb executors; this is the only place cross-mechanism decisions live.
 */
@Logged
public class Superstructure extends SubsystemBase {
  // Passing shot is a fixed lob (legacy hardcoded values, W14).
  private static final double kPassingShooterRps = 95;
  // Manual turret jog deadband (legacy MANUAL_DEADBAND).
  public static final double kManualJoystickDeadband = 0.08;

  private final Drive drive;
  private final Turret turret;
  private final Hood hood;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Vision vision;
  private final ShotCalculator shotCalculator = new ShotCalculator();
  private final TurretAiming aiming = new TurretAiming();

  private final BooleanSupplier shootHeld; // operator RT
  private final BooleanSupplier passHeld; // operator LB
  private final BooleanSupplier unjamHeld; // operator right-stick click
  private final BooleanSupplier manualFeedHeld; // operator LT — vertical feed outside shoot states

  @Logged(importance = Logged.Importance.CRITICAL)
  private RobotState state = RobotState.IDLE;

  @Logged(importance = Logged.Importance.CRITICAL)
  private Goal goal = Goal.IDLE;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean manualMode = false;

  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean intakeRequested = false;

  private Optional<Goal> autoGoalOverride = Optional.empty();
  private final Timer clearingTimer = new Timer();

  public Superstructure(
      Drive drive,
      Turret turret,
      Hood hood,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      Vision vision,
      BooleanSupplier shootHeld,
      BooleanSupplier passHeld,
      BooleanSupplier unjamHeld,
      BooleanSupplier manualFeedHeld) {
    this.drive = drive;
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.vision = vision;
    this.shootHeld = shootHeld;
    this.passHeld = passHeld;
    this.unjamHeld = unjamHeld;
    this.manualFeedHeld = manualFeedHeld;
    aiming.reset(0.0, 0.0);
  }

  @Override
  public void periodic() {
    // The intake axis runs in EVERY mode, including manual (mechanism-level, sensor-free).
    applyIntakeRequest();

    if (manualMode) {
      state = RobotState.MANUAL;
      aiming.rebaseToCurrent(turret.getPositionRot()); // W7: no snap-back on exit
      return;
    }

    goal = autoGoalOverride.orElseGet(this::teleopGoal);
    Conditions conditions = sampleConditions();
    RobotState next = Transitions.next(state, goal, conditions);
    if (next != state) {
      onStateEntry(next);
      state = next;
    }
    applyStateBehavior();
  }

  private Goal teleopGoal() {
    if (unjamHeld.getAsBoolean()) {
      return Goal.UNJAM;
    }
    if (shootHeld.getAsBoolean()) {
      return Goal.SHOOT;
    }
    if (passHeld.getAsBoolean()) {
      return Goal.PASS;
    }
    return Goal.IDLE;
  }

  private Conditions sampleConditions() {
    return new Conditions(
        passHeld.getAsBoolean(),
        aiming.isAimed(),
        shooter.atSpeed(),
        shooter.commandedNonZero(),
        intake.isOut(),
        clearingTimer.hasElapsed(IndexerConstants.kClearingSeconds));
  }

  // One-time actions on entering a state. Continuous behavior lives in applyStateBehavior.
  private void onStateEntry(RobotState next) {
    switch (next) {
      case CLEARING -> {
        clearingTimer.restart();
        shooter.stop();
      }
      case IDLE, INTAKING -> {
        shooter.stop();
        hood.holdCurrentPosition(); // capture-once hold (deliberate change from sag-follow, W32)
      }
      case PASS_AIMING -> shooter.stop();
      default -> {}
    }
  }

  private void applyStateBehavior() {
    double yawDeg = drive.getContinuousYawDeg();
    double omega = drive.getRobotRelativeSpeeds().omegaRadiansPerSecond;

    switch (state) {
      case IDLE, INTAKING -> {
        aiming.trackHub(turret, vision, yawDeg, omega);
        runRestFeed();
      }
      case SPINNING_UP, SHOOTING -> {
        aiming.trackHub(turret, vision, yawDeg, omega);
        commandHubShot();
        indexer.setVelocity(Stage.VERTICAL, IndexerConstants.kVerticalSpeedRps);
        if (state == RobotState.SHOOTING) {
          indexer.setVelocity(Stage.HORIZONTAL, IndexerConstants.kHorizontalSpeedRps);
          indexer.setVelocity(Stage.UPWARD, IndexerConstants.kUpwardSpeedRps);
        } else {
          indexer.stop(Stage.HORIZONTAL);
          indexer.stop(Stage.UPWARD);
        }
      }
      case CLEARING -> {
        aiming.trackHub(turret, vision, yawDeg, omega);
        // Team decision: back every committed ball away from the flywheel at FULL speed.
        indexer.setVelocity(Stage.HORIZONTAL, -IndexerConstants.kHorizontalSpeedRps);
        indexer.setVelocity(Stage.VERTICAL, -IndexerConstants.kVerticalSpeedRps);
        indexer.setVelocity(Stage.UPWARD, -IndexerConstants.kUpwardSpeedRps);
      }
      case PASS_AIMING -> {
        aiming.trackPassing(turret, vision, yawDeg);
        runRestFeed();
      }
      case PASS_SPINNING_UP, PASSING -> {
        aiming.trackPassing(turret, vision, yawDeg);
        hood.setAngle(Rotations.of(HoodConstants.kPassingAngleRot));
        shooter.setVelocity(RotationsPerSecond.of(kPassingShooterRps));
        indexer.setVelocity(Stage.VERTICAL, IndexerConstants.kVerticalSpeedRps);
        if (state == RobotState.PASSING) {
          indexer.setVelocity(Stage.HORIZONTAL, IndexerConstants.kHorizontalSpeedRps);
          indexer.setVelocity(Stage.UPWARD, IndexerConstants.kUpwardSpeedRps);
        } else {
          indexer.stop(Stage.HORIZONTAL);
          indexer.stop(Stage.UPWARD);
        }
      }
      case UNJAMMING -> {
        // Legacy reverseIndexers, verbatim (W19): every stage backward at half speed while the
        // flywheel runs forward flat-out so a pinched ball ejects instead of staying trapped.
        aiming.trackHub(turret, vision, yawDeg, omega);
        indexer.setVelocity(
            Stage.HORIZONTAL, -IndexerConstants.kHorizontalSpeedRps * IndexerConstants.kUnjamFraction);
        indexer.setVelocity(
            Stage.VERTICAL, -IndexerConstants.kVerticalSpeedRps * IndexerConstants.kUnjamFraction);
        indexer.setVelocity(
            Stage.UPWARD, -IndexerConstants.kUpwardSpeedRps * IndexerConstants.kUnjamFraction);
        shooter.setVelocity(RotationsPerSecond.of(ShooterConstants.kMaxRps));
      }
      case MANUAL -> {
        // Unreachable: manualMode returns before the machine runs. Exhaustiveness only.
      }
    }
  }

  // Shooter+hood track the interp tables at the (stale-held) vision distance. When aim is lost
  // the speed HOLDS instead of dropping to zero (W12 fix half A; half B is the gate's
  // shooterCommandedNonZero).
  private void commandHubShot() {
    double distance =
        shotCalculator.effectiveDistance(
            aiming.getDistanceToHub(),
            drive.getRobotRelativeSpeeds(),
            turret.getPosition().getRadians());
    hood.setAngle(Rotations.of(shotCalculator.getHoodAngleRot(distance)));
    shooter.setVelocity(RotationsPerSecond.of(shotCalculator.getFlywheelRps(distance)));
  }

  // Resting feed (W18, verbatim speeds): vertical 26.25 RPS while the intake is out, or full 35
  // while the operator holds LT (legacy manual-feed binding); otherwise stopped.
  private void runRestFeed() {
    if (manualFeedHeld.getAsBoolean()) {
      indexer.setVelocity(Stage.VERTICAL, IndexerConstants.kVerticalSpeedRps);
    } else if (intake.isOut()) {
      indexer.setVelocity(Stage.VERTICAL, IndexerConstants.kIntakingVerticalSpeedRps);
    } else {
      indexer.stop(Stage.VERTICAL);
    }
    indexer.stop(Stage.HORIZONTAL);
    indexer.stop(Stage.UPWARD);
  }

  private void applyIntakeRequest() {
    if (intakeRequested) {
      intake.requestDeploy();
    } else {
      intake.requestStow();
    }
  }

  // ── Operator/auto API ──

  /** Operator A: toggle the intake axis (deploy/stow). Works in every mode including manual. */
  public void toggleIntake() {
    intakeRequested = !intakeRequested;
  }

  public void setIntakeRequested(boolean requested) {
    intakeRequested = requested;
  }

  /** Operator Start: full manual override. Reachable from ANY state; never blockable. */
  public void toggleManualMode() {
    manualMode = !manualMode;
    if (manualMode) {
      enterManual();
    } else {
      exitManualToIdle();
    }
  }

  public boolean isManualMode() {
    return manualMode;
  }

  private void enterManual() {
    shooter.stop();
    indexer.stopAll();
    turret.setTargetPosition(Rotations.of(turret.getPositionRot()));
    hood.holdCurrentPosition();
    state = RobotState.MANUAL;
  }

  // Exit re-sync (design §7): land at IDLE via a defined re-sync, never the pre-manual state.
  private void exitManualToIdle() {
    aiming.reset(turret.getPositionRot(), drive.getContinuousYawDeg());
    hood.holdCurrentPosition();
    shooter.stop();
    indexer.stopAll();
    state = RobotState.IDLE;
    goal = Goal.IDLE;
  }

  /**
   * Re-seed aiming at enable (replaces the legacy command's initialize()). The scoring state also
   * resets to IDLE: a disable mid-CLEARING must not resume an un-timed full-reverse clear.
   * Manual mode deliberately PERSISTS across disable — a sensor-dead robot stays manual.
   */
  public void onEnable() {
    aiming.reset(turret.getPositionRot(), drive.getContinuousYawDeg());
    clearingTimer.stop();
    clearingTimer.reset();
    if (!manualMode) {
      state = RobotState.IDLE;
      goal = Goal.IDLE;
    }
  }

  /** Auto routines hold a goal through this command; releasing it returns control to teleop. */
  public Command goalCommand(Goal heldGoal) {
    return Commands.startEnd(
        () -> autoGoalOverride = Optional.of(heldGoal),
        () -> autoGoalOverride = Optional.empty());
  }

  /** Named command "stopAll" (legacy semantics: stop shooter and all indexers). */
  public void stopAllMechanisms() {
    shooter.stop();
    indexer.stopAll();
  }

  public RobotState getState() {
    return state;
  }

  public TurretAiming getAiming() {
    return aiming;
  }
}
