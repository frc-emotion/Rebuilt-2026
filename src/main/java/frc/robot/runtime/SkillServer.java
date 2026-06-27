package frc.robot.runtime;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import frc.robot.runtime.SkillInterpreter.AxisStatus;
import frc.robot.runtime.SkillInterpreter.InterpreterInputs;
import frc.robot.runtime.SkillInterpreter.ScoringStatus;
import frc.robot.runtime.perception.PerceptionProvider;

/**
 * The NetworkTables skill API — the seam a future coprocessor "brain" will drive the robot over. It
 * is purely a transport: it never decides anything. Today the local driver supplies the request and
 * the coprocessor is absent; tomorrow a coprocessor sets {@code /skills/request/remoteActive} and
 * publishes a skill, and this server hands that to the same interpreter unchanged.
 *
 * <ul>
 *   <li><b>/skills/request/</b> — inbound: {@code skill} (string), {@code manualMode}, {@code
 *       manualFeed}, {@code intakeDeploy} (bools), {@code remoteActive} (bool — when false, the
 *       local driver's request is used).
 *   <li><b>/skills/status/</b> — outbound per-axis: {@code scoring} (idle|running|succeeded|failed|
 *       blocked — the richer scoring status the orchestration brain reads), {@code intake} (ok|
 *       running|fail), {@code activeSkill}, {@code phase}, plus instrumentation topics {@code
 *       reason} (string), {@code blocked} (bool), {@code done} (bool — the skill's done predicate,
 *       advisory; never auto-cancels), {@code timeoutRemaining} (seconds).
 *   <li><b>/state/</b> — outbound state estimate: {@code aimed}, {@code distanceToHub}, {@code
 *       turretTargetRot}, {@code hasPose} (pose estimation is gated off locally).
 * </ul>
 */
public class SkillServer {
  // Local request (set by the LocalSkillDriver and PathPlanner named commands).
  private String localSkill = "idle";
  private boolean localManualMode = false;
  private boolean localManualFeed = false;
  private boolean localIntakeDeploy = false;

  // Inbound (a coprocessor writes these).
  private final StringSubscriber reqSkill;
  private final BooleanSubscriber reqManualMode;
  private final BooleanSubscriber reqManualFeed;
  private final BooleanSubscriber reqIntakeDeploy;
  private final BooleanSubscriber reqRemoteActive;

  // Mirror of the resolved request (so the dashboard always shows what is actually running).
  private final StringPublisher echoSkill;
  private final BooleanPublisher echoRemote;

  // Outbound status.
  private final StringPublisher statusScoring;
  private final StringPublisher statusIntake;
  private final StringPublisher statusActiveSkill;
  private final StringPublisher statusPhase;
  private final StringPublisher statusReason;
  private final BooleanPublisher statusBlocked;
  private final BooleanPublisher statusDone;
  private final DoublePublisher statusTimeoutRemaining;

  // Outbound state estimate.
  private final BooleanPublisher stateAimed;
  private final DoublePublisher stateDistanceToHub;
  private final DoublePublisher stateTurretTargetRot;
  private final BooleanPublisher stateHasPose;

  public SkillServer() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    NetworkTable req = nt.getTable("skills").getSubTable("request");
    NetworkTable status = nt.getTable("skills").getSubTable("status");
    NetworkTable state = nt.getTable("state");

    reqSkill = req.getStringTopic("skill").subscribe("idle");
    reqManualMode = req.getBooleanTopic("manualMode").subscribe(false);
    reqManualFeed = req.getBooleanTopic("manualFeed").subscribe(false);
    reqIntakeDeploy = req.getBooleanTopic("intakeDeploy").subscribe(false);
    reqRemoteActive = req.getBooleanTopic("remoteActive").subscribe(false);

    echoSkill = req.getStringTopic("resolvedSkill").publish();
    echoRemote = req.getBooleanTopic("remoteActiveEcho").publish();

    statusScoring = status.getStringTopic("scoring").publish();
    statusIntake = status.getStringTopic("intake").publish();
    statusActiveSkill = status.getStringTopic("activeSkill").publish();
    statusPhase = status.getStringTopic("phase").publish();
    statusReason = status.getStringTopic("reason").publish();
    statusBlocked = status.getBooleanTopic("blocked").publish();
    statusDone = status.getBooleanTopic("done").publish();
    statusTimeoutRemaining = status.getDoubleTopic("timeoutRemaining").publish();

    stateAimed = state.getBooleanTopic("aimed").publish();
    stateDistanceToHub = state.getDoubleTopic("distanceToHub").publish();
    stateTurretTargetRot = state.getDoubleTopic("turretTargetRot").publish();
    stateHasPose = state.getBooleanTopic("hasPose").publish();
  }

  // ── Local request setters (LocalSkillDriver + PathPlanner named commands) ──

  /** invoke(skill): request a scoring skill locally (held until changed/canceled). */
  public void invoke(String skill) {
    localSkill = skill;
  }

  /** cancel(): drop back to idle locally. */
  public void cancel() {
    localSkill = "idle";
  }

  public void setManualMode(boolean manualMode) {
    localManualMode = manualMode;
  }

  public void setManualFeed(boolean manualFeed) {
    localManualFeed = manualFeed;
  }

  public void setIntakeDeploy(boolean deploy) {
    localIntakeDeploy = deploy;
  }

  public boolean localManualMode() {
    return localManualMode;
  }

  public boolean localIntakeDeploy() {
    return localIntakeDeploy;
  }

  /** Resolve the active request for this loop: remote if a coprocessor is driving, else local. */
  public InterpreterInputs resolve() {
    boolean remote = reqRemoteActive.get();
    InterpreterInputs inputs =
        remote
            ? new InterpreterInputs(
                reqSkill.get(), reqManualMode.get(), reqManualFeed.get(), reqIntakeDeploy.get())
            : new InterpreterInputs(
                localSkill, localManualMode, localManualFeed, localIntakeDeploy);
    echoSkill.set(inputs.scoringSkill());
    echoRemote.set(remote);
    return inputs;
  }

  /** Publish status + the state estimate seam (call once per loop after the interpreter runs). */
  public void publish(SkillInterpreter interpreter, PerceptionProvider perception) {
    ScoringStatus scoring = interpreter.scoringStatus();
    statusScoring.set(name(scoring));
    statusIntake.set(name(interpreter.intakeStatus()));
    statusActiveSkill.set(interpreter.activeScoringSkill());
    statusPhase.set(interpreter.phase().name());
    statusReason.set(interpreter.scoringReason());
    statusBlocked.set(scoring == ScoringStatus.BLOCKED);
    statusDone.set(interpreter.isDone());
    statusTimeoutRemaining.set(interpreter.timeoutRemainingSeconds());

    stateAimed.set(interpreter.aiming().isAimed());
    stateDistanceToHub.set(interpreter.aiming().getDistanceToHub());
    stateTurretTargetRot.set(interpreter.aiming().getTargetPositionRot());
    stateHasPose.set(perception.getEstimatedPose().isPresent());
  }

  private static String name(AxisStatus status) {
    return switch (status) {
      case OK -> "ok";
      case RUNNING -> "running";
      case FAIL -> "fail";
    };
  }

  private static String name(ScoringStatus status) {
    return switch (status) {
      case IDLE -> "idle";
      case RUNNING -> "running";
      case SUCCEEDED -> "succeeded";
      case FAILED -> "failed";
      case BLOCKED -> "blocked";
    };
  }
}
