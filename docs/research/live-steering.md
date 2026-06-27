# Live Velocity Steering and Mid-Match Re-Routing

> Audience: FRC team members who know WPILib/Java but have not driven this robot's
> drivetrain from anything other than the joystick or a pre-drawn PathPlanner auto.
>
> Status: **EXPLAINER + DESIGN + CODE SKETCH. Nothing here is applied.** The drivetrain
> (`subsystems/drive/CommandSwerveDrivetrain.java`, `TunerConstants.java`) is frozen,
> byte-identical Phoenix Tuner X output and must not change. The only proposed code change
> is a small addition to the hand-written adapter `Drive.java`.

---

## TL;DR

A future "autonomy brain" (an orchestration layer that decides where the robot should go)
cannot today say *"drive at these ChassisSpeeds right now"* or *"compute a fresh route to
this pose mid-match."* Both capabilities already exist **inside** the drivetrain stack — they
are just not exposed:

1. The CTRE request that accepts arbitrary chassis speeds (`SwerveRequest.ApplyRobotSpeeds`)
   is created as a **private local variable** inside `Drive.configureAutoBuilder()` and only
   ever fires while a pre-loaded PathPlanner path is playing.
2. PathPlanner's on-the-fly pathfinding (`AutoBuilder.pathfindToPose` /
   `pathfindThenFollowPath`) is fully configured (we call `AutoBuilder.configure(...)` already)
   but never invoked anywhere.

The fix is two small **additive** methods on `Drive` — no generated file is touched.

---

## 1) Why the gap exists

Today there are exactly two ways the wheels turn:

| Path | Entry point | Who supplies the speeds |
| --- | --- | --- |
| Teleop | `Drive.teleopDriveCommand(leftY, leftX, rightX)` | Joystick suppliers, read every loop |
| Pre-drawn auto | A `.auto` file → `AutoBuilder` → `PPHolonomicDriveController` | PathPlanner trajectory sampler |

Look at `Drive.java`:

- `teleopDriveCommand` (lines ~39-48) wraps a `SwerveRequest.FieldCentric` and reads three
  `DoubleSupplier`s. It is **field-centric** and **negates all three axes** (WPILib convention:
  forward is `-leftY`, left is `-leftX`, CCW is `-rightX`). It multiplies by
  `DriveConstants.kMaxSpeedMps` (5.85 m/s) and `kMaxAngularRateRadPerSec` (4.712 rad/s).
- `configureAutoBuilder()` (lines ~101-128) builds the speeds consumer PathPlanner needs:

  ```java
  SwerveRequest.ApplyRobotSpeeds pathRequest = new SwerveRequest.ApplyRobotSpeeds(); // <-- private local
  AutoBuilder.configure(
      () -> drivetrain.getState().Pose,            // pose supplier
      drivetrain::resetPose,                       // pose reset
      () -> drivetrain.getState().Speeds,          // robot-relative speeds supplier
      (speeds, feedforwards) ->                     // <-- THE consumer that takes ChassisSpeeds
          drivetrain.setControl(
              pathRequest
                  .withSpeeds(speeds)
                  .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                  .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
      new PPHolonomicDriveController(...),
      config,                                       // RobotConfig.fromGUISettings()
      () -> isRedAlliance(),
      drivetrain);
  ```

**The gap:** that `(speeds, feedforwards) -> drivetrain.setControl(...)` lambda is *exactly* the
"drive these ChassisSpeeds now" primitive we want — but it is buried inside `AutoBuilder.configure`,
captured by a private local `pathRequest`, and PathPlanner is the only caller. It fires **only**
while a path command is scheduled. There is:

- **No public method** for arbitrary code to push a `ChassisSpeeds` for one loop.
- **No method** to ask for a freshly-computed route to a pose chosen at runtime.

`Drive` deliberately exposes only Command-returning methods plus getters
(`getPose`, `getRobotRelativeSpeeds`, `getContinuousYawDeg`, ...). That is the right shape for
the rest of the robot — we just need to add the two missing primitives.

---

## 2) The simple fix — a public `runVelocity(ChassisSpeeds)`

Add a reusable `ApplyRobotSpeeds` request as a field (mirroring how `teleopRequest`,
`brakeRequest`, and `idleRequest` are fields) and a method that pushes one set of speeds.

`ApplyRobotSpeeds` interprets its `ChassisSpeeds` as **robot-relative** (vx = forward,
vy = left, omega = CCW, in m/s and rad/s). If the brain thinks in field coordinates, either
convert with `ChassisSpeeds.fromFieldRelativeSpeeds(...)` before calling, or use the
field-centric request shown in the variant below.

### Proposed addition to `Drive.java` (NOT YET APPLIED)

```java
// === NEW FIELD (add alongside teleopRequest / brakeRequest / idleRequest) ===
/**
 * Reusable request for programmatic velocity steering. Robot-relative:
 * ChassisSpeeds vx = forward (m/s), vy = left (m/s), omega = CCW (rad/s).
 * Same request type the PathPlanner consumer uses, so behavior matches autos.
 */
private final SwerveRequest.ApplyRobotSpeeds velocityRequest =
    new SwerveRequest.ApplyRobotSpeeds();

// Optional: a field-centric variant if the brain prefers field-frame speeds.
private final SwerveRequest.FieldCentric fieldVelocityRequest =
    new SwerveRequest.FieldCentric()
        .withDriveRequestType(
            com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

// === NEW METHODS ===

/**
 * Drive at the given ROBOT-RELATIVE chassis speeds for this loop. Call every ~20 ms;
 * the request latches the last value, so a single call holds until the next one.
 * Intended for an external orchestration loop, not a default command.
 */
public void runVelocity(ChassisSpeeds robotRelativeSpeeds) {
  drivetrain.setControl(velocityRequest.withSpeeds(robotRelativeSpeeds));
}

/**
 * Drive at FIELD-RELATIVE speeds (vx = +X field forward, vy = +Y field left, omega CCW).
 * Note the sign/units convention matches teleop (WPILib field-centric), NOT the negated
 * raw-joystick axes — pass already-signed velocities, not stick values.
 */
public void runFieldVelocity(double vxMps, double vyMps, double omegaRadPerSec) {
  drivetrain.setControl(
      fieldVelocityRequest
          .withVelocityX(vxMps)
          .withVelocityY(vyMps)
          .withRotationalRate(omegaRadPerSec));
}

/** Stop the drivetrain immediately (zero robot-relative speeds). */
public void stop() {
  drivetrain.setControl(velocityRequest.withSpeeds(new ChassisSpeeds()));
}
```

That is the whole change. It reuses the exact request/`setControl` mechanism the autos already
trust, so a velocity command from the brain behaves identically to a velocity command from a path.

### How the brain calls it each loop

`runVelocity` is a plain method, not a Command — call it directly from whatever periodic loop
the autonomy brain runs in (e.g. `robotPeriodic`, a Notifier, or the orchestration tick):

```java
// inside the brain's ~20 ms loop
ChassisSpeeds desired = brain.computeRobotRelativeSpeeds();
drive.runVelocity(desired);
```

### Who wins vs the default teleop command (requirements)

This is the one subtlety. In this repo the **subsystem handle is the generated
`CommandSwerveDrivetrain`** (`Drive.subsystem()`), and the teleop drive is installed as that
subsystem's **default command** (it is `drivetrain.applyRequest(...)`, which requires the
subsystem). The CommandScheduler rule: *a subsystem runs its default command whenever no other
command requires it.*

`runVelocity` calls `drivetrain.setControl(...)` **outside** the command system. That means:

- If the **default teleop command is scheduled** (the normal case), it also calls `setControl`
  every loop. Two writers race; **the last `setControl` of the loop wins.** Whoever runs later in
  the loop order silently overrides the other — fragile and order-dependent. Do **not** rely on this.
- The correct pattern is to make the brain's authority explicit by wrapping `runVelocity` in a
  Command that **requires the drivetrain subsystem**, so the scheduler suspends the default command
  while the brain drives:

  ```java
  /** A command that hands velocity control to the brain until cancelled. */
  public Command runVelocityCommand(Supplier<ChassisSpeeds> robotRelativeSpeeds) {
    return drivetrain.run(() -> drivetrain.setControl(
        velocityRequest.withSpeeds(robotRelativeSpeeds.get())));
    // drivetrain.run(...) requires the subsystem, so teleop's default command is suspended
    // for the command's lifetime, and resumes automatically when this command ends.
  }
  ```

  Schedule `runVelocityCommand(brain::computeSpeeds)` when the brain takes over; cancel it (or let
  a button/trigger interrupt it) to give the driver back control. This is the same ownership model
  PathPlanner uses — its path commands require the subsystem too. **Recommendation: ship the
  Command-wrapped form as the primary API**, and keep the bare `runVelocity` only for code that
  already owns the subsystem (e.g. inside another drivetrain-requiring command).

### Feeding it from a simple P-controller-to-a-target

A minimal "go to this pose" loop without PathPlanner — useful for short corrections or as a
fallback when no navgrid route is needed:

```java
// kP gains are illustrative; tune on carpet. Clamp to max speeds.
Pose2d current = drive.getPose();
double kP = 2.0, kPtheta = 3.0;

double vx = kP * (targetX - current.getX());
double vy = kP * (targetY - current.getY());
double omega = kPtheta * targetRot.minus(current.getRotation()).getRadians();

// clamp to the robot's limits
double speed = Math.hypot(vx, vy);
if (speed > DriveConstants.kMaxSpeedMps) {
  double s = DriveConstants.kMaxSpeedMps / speed;
  vx *= s; vy *= s;
}
omega = MathUtil.clamp(omega, -DriveConstants.kMaxAngularRateRadPerSec,
                              DriveConstants.kMaxAngularRateRadPerSec);

// vx/vy here are FIELD-relative (computed from field-frame pose error):
drive.runFieldVelocity(vx, vy, omega);
```

For robot-relative control instead, build a `ChassisSpeeds` and call `runVelocity(...)`.

---

## 3) Re-routing / on-the-fly paths

For real obstacle-aware re-routing (not just a straight-line P-controller), use PathPlanner's
pathfinding. It plans a path from the robot's *current* pose to a target, around the static field
obstacles described in `navgrid.json`, and follows it with the same holonomic controller the autos
use.

Two relevant calls (Java, current API — see Context7 notes below):

- `AutoBuilder.pathfindToPose(Pose2d target, PathConstraints constraints)` — plan + drive to a
  pose chosen at runtime. Overloads add `double goalEndVelocity` and `double rotationDelayDistance`.
- `AutoBuilder.pathfindThenFollowPath(PathPlannerPath path, PathConstraints constraints)` —
  pathfind to the **start** of a pre-drawn path, then follow that path exactly. Good for "get onto
  my scoring lane from wherever I am, then run the known-good lane."

Both return a `Command` and **require an already-configured `AutoBuilder`** — which we have, because
`Drive.configureAutoBuilder()` calls `AutoBuilder.configure(...)` at startup. They also read
`deploy/pathplanner/navgrid.json` for the obstacle grid (present in this repo) and the
`RobotConfig` from `settings.json` (already loaded via `RobotConfig.fromGUISettings()`).

### Proposed wrapper on `Drive.java` (NOT YET APPLIED)

```java
import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;  // for the pathfindThenFollow variant

/** Default constraints for runtime pathfinding. Tune; can also accept as a parameter. */
private static final PathConstraints kPathfindConstraints =
    new PathConstraints(
        3.0,                                   // max linear vel (m/s)
        3.0,                                   // max linear accel (m/s^2)
        Units.degreesToRadians(540),           // max angular vel (rad/s)
        Units.degreesToRadians(720));          // max angular accel (rad/s^2)

/**
 * Build a command that pathfinds from the current pose to {@code target}, avoiding the
 * static obstacles in navgrid.json. Returns a Command that requires the drivetrain, so it
 * cooperates with the scheduler (interrupts teleop, is interruptible itself).
 */
public Command pathfindToPoseCommand(Pose2d target) {
  return AutoBuilder.pathfindToPose(target, kPathfindConstraints);
  // AutoBuilder already has the drivetrain as a requirement from configure(...).
}
```

`AutoBuilder.pathfindToPose` returns a command that already requires the drivetrain subsystem (it
was registered in `AutoBuilder.configure(...)`), so it interrupts the teleop default command for
its lifetime — same ownership story as section 2.

### Buildable-in-sim now vs blocked on hardware

| Capability | Status | Depends on |
| --- | --- | --- |
| Pathfind around **static field walls/obstacles** | **Buildable now** | `navgrid.json` (present), `RobotConfig` (loaded). The navgrid encodes the fixed field geometry from the PathPlanner GUI. |
| Pathfind around **other robots / live field debris** | **Not yet** | A live obstacle source feeding PathPlanner's dynamic obstacle API. **No depth-camera obstacle data exists in this repo yet** — `runtime.perception.PerceptionProvider`/vision only does AprilTags. |

**Be explicit with the team:** today's pathfinding dodges only the *static* field as drawn in
`navgrid.json`. It will happily drive straight through another robot, because nothing tells it
another robot is there. Dynamic avoidance requires (a) a depth/3D sensor or detector that produces
obstacle positions each loop, and (b) wiring those into PathPlanner's dynamic-obstacle setter every
loop. Both are future work and neither exists yet. Until then, treat pathfinding as "smart routing
around the field," not "collision avoidance."

---

## 4) Testing `runVelocity` in simulation today (no hardware)

The CTRE generated drivetrain includes a physics sim (`CommandSwerveDrivetrain` runs
`updateSimState` in simulation), so `runVelocity` is fully exercisable on a laptop:

1. **Launch sim:** `./gradlew simulateJava` (the normal sim entry point for this repo).
2. **Drive Station + Glass/AdvantageScope:** enable in Teleop or Autonomous. Watch the pose from
   `drive.getPose()` and the reported speeds from `drive.getRobotRelativeSpeeds()`.
3. **Manual smoke test:** from a temporary trigger or the sim console, call
   `drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0))` and confirm the simulated pose's X advances
   at ~1 m/s and that zeroing the speeds (`drive.stop()`) halts it.
4. **P-controller test:** schedule the section-2 P-loop toward a fixed target pose and confirm the
   sim pose converges and settles without oscillation; tune `kP`/`kPtheta` here, not on carpet.
5. **Unit/integration test:** a JUnit test can construct `Drive`, step the scheduler/sim, call
   `runVelocity`, and assert `getRobotRelativeSpeeds()` tracks the command. Per this repo's rules,
   any test touching WPILib sim must call `HAL.initialize(500, 0)` in `@BeforeAll`, and any test
   constructing the drivetrain subsystem must
   `CommandScheduler.getInstance().unregisterAllSubsystems()` in setup **and** teardown (the
   drivetrain is a `SubsystemBase` and self-registers). Field-centric output also depends on a seeded
   gyro — call `seedFieldCentricCommand()`/`seedFieldCentric` first or results are heading-relative
   to an unseeded zero.

Pathfinding (`pathfindToPoseCommand`) is also sim-testable: schedule it toward a reachable pose and
watch the generated trajectory in AdvantageScope's PathPlanner overlay. It uses the same
`navgrid.json` in sim as on the robot.

---

## 5) Risks and caveats

- **Do not touch the generated drivetrain.** All of this lives in the hand-written `Drive.java`
  adapter. `TunerConstants.java` and `CommandSwerveDrivetrain.java` stay byte-identical (Spotless
  excludes them; verify with `git diff`). `runVelocity` only adds a field + method to `Drive`; it
  reuses the same `SwerveRequest`/`setControl` plumbing the autos already use, so risk is low.
- **The 5.44 vs 5.85 m/s max-speed mismatch is real and matters here.** `DriveConstants.kMaxSpeedMps`
  = 5.85 (from `TunerConstants.kSpeedAt12Volts`), but `deploy/pathplanner/settings.json` declares
  `maxDriveSpeed = 5.44` and that is what `RobotConfig.fromGUISettings()` feeds PathPlanner. So:
  teleop and any clamp you write against `kMaxSpeedMps` allow 5.85; PathPlanner's `RobotConfig`
  (used for feedforwards and `desaturateWheelSpeeds`) assumes 5.44. When clamping `runVelocity`
  inputs, **clamp to the more conservative 5.44** if you want behavior consistent with the path
  follower, or knowingly accept that hand-driven velocities can exceed what PathPlanner believes the
  robot can do. Don't silently mix the two. (This mismatch is documented in `DriveConstants` and
  CLAUDE.md as a known second-source-of-truth issue — not something to "fix" casually, since the
  tuned PathPlanner numbers are sacred.)
- **Motor safety / watchdog while feeding live commands.** `setControl` requests do **not** time out
  on their own the way `MotorSafety`-enabled outputs do — a `SwerveRequest` latches the last value
  and the modules keep driving until the next `setControl`. If the brain's loop stalls, hangs, or the
  controlling command ends without a final stop, **the robot keeps moving at the last commanded
  speed.** Mitigations: (1) prefer the **Command-wrapped** form so ending/interrupting the command
  returns control to teleop (which writes zeros on no input via its deadband); (2) have the brain
  send `drive.stop()` (zero speeds) as the last action when it relinquishes control; (3) consider a
  staleness guard — if no fresh brain command arrived in N loops, command `stop()`. Also note the
  drivetrain only actuates when the robot is enabled; disabling cuts output regardless.
- **Pathfinding ≠ collision avoidance (repeat).** See section 3 — only static field geometry is
  avoided until a live obstacle feed exists.

---

## What to build now vs later

| Item | Now / Later | Notes |
| --- | --- | --- |
| `runVelocity(ChassisSpeeds)` + `runFieldVelocity(...)` + `stop()` on `Drive` | **Now** | ~15 lines, additive, reuses the autos' `ApplyRobotSpeeds` path. |
| Command-wrapped `runVelocityCommand(Supplier<ChassisSpeeds>)` | **Now** | The safe, scheduler-aware way to give the brain authority. Prefer over bare `runVelocity`. |
| P-controller-to-pose helper | **Now** | Pure math on `getPose()`; great first sim test. |
| `pathfindToPoseCommand(Pose2d)` wrapper | **Now** | `AutoBuilder` already configured; navgrid present. Avoids static field only. |
| `pathfindThenFollowPath` wrapper | **Now** | Same prerequisites; good for snapping onto known scoring lanes. |
| Dynamic obstacle avoidance (other robots) | **Later** | Blocked on a depth-camera / detector obstacle feed that does not exist yet. |
| Staleness/watchdog guard on live velocity | **Now (cheap, do it with the feature)** | Prevents runaway if the brain loop stalls. |

---

## Context7-verified API notes

Verified against Context7 on 2026-06-26.

**PathPlanner** — `/mjansen4857/pathplanner` (High reputation; source: `pplib-Pathfinding`).
Java signatures confirmed:

- `AutoBuilder.pathfindToPose(Pose2d targetPose, PathConstraints constraints)` — returns a `Command`.
  Overloads add `double goalEndVelocity` (m/s, default 0.0) and `double rotationDelayDistance`
  (m, default 0.0). For holonomic drivetrains the target pose's rotation is the goal heading.
- `AutoBuilder.pathfindThenFollowPath(PathPlannerPath path, PathConstraints constraints)` — returns
  a `Command`; pathfinds to the path's start, then follows it. Overload adds `rotationDelayDistance`.
- `PathConstraints(double maxVelMps, double maxAccelMps2, double maxAngVelRadPerSec,
  double maxAngAccelRadPerSec2)`.
- Both require `AutoBuilder.configure(...)` to have run (done here in `configureAutoBuilder()`) and
  read `navgrid.json` for obstacle data.

**CTRE Phoenix 6 Swerve** — `/crosstheroadelec/phoenix6-documentation` (High reputation; source:
`api-reference/mechanisms/swerve/swerve-requests.md`). Confirmed:

- `SwerveRequest.ApplyRobotSpeeds` with `.withSpeeds(ChassisSpeeds)` (robot-relative), plus the
  `.withWheelForceFeedforwardsX/Y(...)` used by the PathPlanner consumer. Applied via
  `drivetrain.setControl(request)`.
- `SwerveRequest.FieldCentric` builder: `.withVelocityX(mps)` (WPILib +X forward),
  `.withVelocityY(mps)` (+Y left), `.withRotationalRate(radPerSec)` (CCW positive),
  plus `.withDeadband(...)`, `.withRotationalDeadband(...)`, `.withDriveRequestType(...)`. The docs
  reiterate the WPILib convention and that teleop negates raw joystick axes
  (`-getLeftY()`, `-getLeftX()`, `-getRightX()`) — matching this repo's `teleopDriveCommand`.
- `SwerveRequest.RobotCentric` exists with the same `withVelocityX/Y/RotationalRate` builders for
  robot-frame velocity if you prefer it over `ApplyRobotSpeeds` (the difference: `ApplyRobotSpeeds`
  takes a whole `ChassisSpeeds` and supports wheel-force feedforwards; `RobotCentric` takes the
  three scalars). For the brain, `ApplyRobotSpeeds` is the better match since the brain produces a
  `ChassisSpeeds`.
- `drivetrain.setControl(SwerveRequest)` is the single apply path; requests latch their last value.
