package frc.robot.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Every legal transition, key illegal-transition rejections, atomic completion under preemption,
 * and return-to-rest. IDs (T#) match REFACTOR_DESIGN.md §2 and docs/superstructure.md.
 */
class TransitionsTest {

  // Conditions builder: everything false/closed by default, opt in per test.
  private static Conditions cond() {
    return new Conditions(false, false, false, false, false, false);
  }

  private static Conditions gateOpen() {
    return new Conditions(false, true, true, true, false, false);
  }

  private static Conditions with(
      boolean pass, boolean aimed, boolean atSpeed, boolean nonZero, boolean intake, boolean cleared) {
    return new Conditions(pass, aimed, atSpeed, nonZero, intake, cleared);
  }

  // ── Rest states (T0–T4) ──

  @Test
  void idleToIntakingOnToggle() {
    assertEquals(
        RobotState.INTAKING,
        Transitions.next(RobotState.IDLE, Goal.IDLE, with(false, false, false, false, true, false)));
  }

  @Test
  void intakingToIdleOnToggle() {
    assertEquals(RobotState.IDLE, Transitions.next(RobotState.INTAKING, Goal.IDLE, cond()));
  }

  @Test
  void idleToSpinningUpOnShoot() {
    assertEquals(RobotState.SPINNING_UP, Transitions.next(RobotState.IDLE, Goal.SHOOT, cond()));
  }

  @Test
  void intakingToSpinningUpKeepsIntakeDeployed() {
    // The intake axis never blocks a scoring request; deployment carries via the condition.
    assertEquals(
        RobotState.SPINNING_UP,
        Transitions.next(RobotState.INTAKING, Goal.SHOOT, with(false, false, false, false, true, false)));
  }

  @Test
  void idleToPassSpinningUpWithPassSelected() {
    assertEquals(
        RobotState.PASS_SPINNING_UP,
        Transitions.next(RobotState.IDLE, Goal.SHOOT, with(true, false, false, false, false, false)));
  }

  @Test
  void idleToPassAimingOnPass() {
    assertEquals(RobotState.PASS_AIMING, Transitions.next(RobotState.IDLE, Goal.PASS, cond()));
  }

  @Test
  void idleToUnjamming() {
    assertEquals(RobotState.UNJAMMING, Transitions.next(RobotState.IDLE, Goal.UNJAM, cond()));
  }

  // ── SPINNING_UP (T5–T7) ──

  @Test
  void spinningUpHoldsUntilFullGate() {
    // The W12 regression test: aimed alone, speed alone, or a ZERO setpoint must never fire.
    assertEquals(
        RobotState.SPINNING_UP,
        Transitions.next(RobotState.SPINNING_UP, Goal.SHOOT, with(false, true, false, false, false, false)));
    assertEquals(
        RobotState.SPINNING_UP,
        Transitions.next(RobotState.SPINNING_UP, Goal.SHOOT, with(false, false, true, true, false, false)));
    assertEquals(
        RobotState.SPINNING_UP,
        Transitions.next(RobotState.SPINNING_UP, Goal.SHOOT, with(false, true, true, false, false, false)));
  }

  @Test
  void spinningUpToShootingWhenGateSatisfied() {
    assertEquals(RobotState.SHOOTING, Transitions.next(RobotState.SPINNING_UP, Goal.SHOOT, gateOpen()));
  }

  @Test
  void spinningUpReleaseSkipsClearing() {
    // No balls committed yet — releasing the trigger goes straight to rest, no clear (T7).
    assertEquals(RobotState.IDLE, Transitions.next(RobotState.SPINNING_UP, Goal.IDLE, cond()));
  }

  @Test
  void spinningUpToPassChainOnLb() {
    assertEquals(
        RobotState.PASS_SPINNING_UP,
        Transitions.next(RobotState.SPINNING_UP, Goal.SHOOT, with(true, false, false, false, false, false)));
  }

  // ── SHOOTING (T8–T10u) ──

  @Test
  void shootingReclosesOnSpeedDroop() {
    assertEquals(
        RobotState.SPINNING_UP,
        Transitions.next(RobotState.SHOOTING, Goal.SHOOT, with(false, true, false, true, false, false)));
  }

  @Test
  void shootingReclosesOnAimLoss() {
    assertEquals(
        RobotState.SPINNING_UP,
        Transitions.next(RobotState.SHOOTING, Goal.SHOOT, with(false, false, true, true, false, false)));
  }

  @Test
  void shootingReleaseEntersClearing() {
    assertEquals(RobotState.CLEARING, Transitions.next(RobotState.SHOOTING, Goal.IDLE, gateOpen()));
    assertEquals(RobotState.CLEARING, Transitions.next(RobotState.SHOOTING, Goal.PASS, gateOpen()));
  }

  @Test
  void unjamPreemptsShooting() {
    assertEquals(RobotState.UNJAMMING, Transitions.next(RobotState.SHOOTING, Goal.UNJAM, gateOpen()));
  }

  // ── CLEARING (T22–T24, the atomic transition) ──

  @Test
  void clearingDefersIdleAndPassGoalsUntilElapsed() {
    // Atomic completion under preemption: goal flaps IDLE/PASS mid-clear, state stays CLEARING.
    assertEquals(RobotState.CLEARING, Transitions.next(RobotState.CLEARING, Goal.IDLE, cond()));
    assertEquals(RobotState.CLEARING, Transitions.next(RobotState.CLEARING, Goal.PASS, cond()));
  }

  @Test
  void clearingExitsToIdleOrIntakingPerIntakeToggle() {
    assertEquals(
        RobotState.IDLE,
        Transitions.next(RobotState.CLEARING, Goal.IDLE, with(false, false, false, false, false, true)));
    assertEquals(
        RobotState.INTAKING,
        Transitions.next(RobotState.CLEARING, Goal.IDLE, with(false, false, false, false, true, true)));
  }

  @Test
  void clearingExitsToPassAimingWhenLbHeld() {
    assertEquals(
        RobotState.PASS_AIMING,
        Transitions.next(RobotState.CLEARING, Goal.PASS, with(true, false, false, false, false, true)));
  }

  @Test
  void clearingReentersShootChain() {
    assertEquals(RobotState.SPINNING_UP, Transitions.next(RobotState.CLEARING, Goal.SHOOT, cond()));
    assertEquals(
        RobotState.PASS_SPINNING_UP,
        Transitions.next(RobotState.CLEARING, Goal.SHOOT, with(true, false, false, false, false, false)));
  }

  @Test
  void unjamPreemptsClearing() {
    assertEquals(RobotState.UNJAMMING, Transitions.next(RobotState.CLEARING, Goal.UNJAM, cond()));
  }

  // ── Pass chain (T11–T18u) ──

  @Test
  void passAimingToPassSpinningUpOnShoot() {
    assertEquals(
        RobotState.PASS_SPINNING_UP,
        Transitions.next(RobotState.PASS_AIMING, Goal.SHOOT, with(true, false, false, false, false, false)));
  }

  @Test
  void passAimingReturnsToRest() {
    assertEquals(RobotState.IDLE, Transitions.next(RobotState.PASS_AIMING, Goal.IDLE, cond()));
  }

  @Test
  void passChainRequiresOnlySpeed() {
    // Legacy quirk, preserved: passing never consults aimed.
    assertEquals(
        RobotState.PASSING,
        Transitions.next(
            RobotState.PASS_SPINNING_UP, Goal.SHOOT, with(true, false, true, true, false, false)));
  }

  @Test
  void lbReleaseMidPassSpinFallsBackToHubChain() {
    assertEquals(
        RobotState.SPINNING_UP,
        Transitions.next(RobotState.PASS_SPINNING_UP, Goal.SHOOT, with(false, false, true, true, false, false)));
  }

  @Test
  void passingReclosesOnSpeedDroop() {
    assertEquals(
        RobotState.PASS_SPINNING_UP,
        Transitions.next(RobotState.PASSING, Goal.SHOOT, with(true, false, false, false, false, false)));
  }

  @Test
  void passingReleaseEntersClearing() {
    assertEquals(
        RobotState.CLEARING,
        Transitions.next(RobotState.PASSING, Goal.PASS, with(true, false, true, true, false, false)));
  }

  // ── Remaining direct row coverage (gate-report gaps) ──

  @Test
  void shootingToPassChainOnLbMidVolley() { // T9
    assertEquals(
        RobotState.PASS_SPINNING_UP,
        Transitions.next(RobotState.SHOOTING, Goal.SHOOT, with(true, true, true, true, false, false)));
  }

  @Test
  void passAimingToUnjamming() { // T13
    assertEquals(RobotState.UNJAMMING, Transitions.next(RobotState.PASS_AIMING, Goal.UNJAM, cond()));
  }

  @Test
  void intakingSharesAllOutboundTransitions() { // T2/T3/T4 from INTAKING
    Conditions deployed = with(true, false, false, false, true, false);
    assertEquals(RobotState.PASS_SPINNING_UP, Transitions.next(RobotState.INTAKING, Goal.SHOOT, deployed));
    assertEquals(RobotState.PASS_AIMING, Transitions.next(RobotState.INTAKING, Goal.PASS, deployed));
    assertEquals(RobotState.UNJAMMING, Transitions.next(RobotState.INTAKING, Goal.UNJAM, deployed));
  }

  @Test
  void spinUpExitsResolvePerGoal() { // T7 PASS/UNJAM variants
    assertEquals(RobotState.PASS_AIMING, Transitions.next(RobotState.SPINNING_UP, Goal.PASS, cond()));
    assertEquals(RobotState.UNJAMMING, Transitions.next(RobotState.SPINNING_UP, Goal.UNJAM, cond()));
  }

  @Test
  void passSpinUpExitsResolvePerGoal() { // T16 PASS/UNJAM variants
    assertEquals(RobotState.PASS_AIMING, Transitions.next(RobotState.PASS_SPINNING_UP, Goal.PASS, cond()));
    assertEquals(RobotState.UNJAMMING, Transitions.next(RobotState.PASS_SPINNING_UP, Goal.UNJAM, cond()));
  }

  @Test
  void passingIdleReleaseAlsoEntersClearing() { // T18 with goal IDLE
    assertEquals(RobotState.CLEARING, Transitions.next(RobotState.PASSING, Goal.IDLE, gateOpen()));
  }

  @Test
  void passAimingReturnsToIntakingWhenDeployed() { // T12 landing on INTAKING
    assertEquals(
        RobotState.INTAKING,
        Transitions.next(RobotState.PASS_AIMING, Goal.IDLE, with(false, false, false, false, true, false)));
  }

  @Test
  void passingLbReleaseFallsBackToHubChain() { // T15 mirror (documented in diagram)
    assertEquals(
        RobotState.SPINNING_UP,
        Transitions.next(RobotState.PASSING, Goal.SHOOT, with(false, false, true, true, false, false)));
  }

  // ── UNJAMMING (T19) ──

  @Test
  void unjammingReturnsToRestOnRelease() {
    assertEquals(RobotState.IDLE, Transitions.next(RobotState.UNJAMMING, Goal.IDLE, cond()));
    assertEquals(
        RobotState.INTAKING,
        Transitions.next(RobotState.UNJAMMING, Goal.IDLE, with(false, false, false, false, true, false)));
    assertEquals(
        RobotState.PASS_AIMING,
        Transitions.next(RobotState.UNJAMMING, Goal.PASS, with(true, false, false, false, false, false)));
  }

  // ── Illegal-transition rejections ──

  @Test
  void shootingUnreachableFromIdleDirectly() {
    // Even with the full gate satisfied, IDLE+SHOOT lands in SPINNING_UP, never SHOOTING.
    assertNotEquals(RobotState.SHOOTING, Transitions.next(RobotState.IDLE, Goal.SHOOT, gateOpen()));
  }

  @Test
  void passingUnreachableWithoutSpeed() {
    assertNotEquals(
        RobotState.PASSING,
        Transitions.next(
            RobotState.PASS_SPINNING_UP, Goal.SHOOT, with(true, true, false, true, false, false)));
  }

  @ParameterizedTest
  @EnumSource(
      value = RobotState.class,
      names = {"IDLE", "INTAKING", "SPINNING_UP", "PASS_AIMING", "PASS_SPINNING_UP", "UNJAMMING"})
  void clearingUnreachableExceptFromShootingOrPassing(RobotState from) {
    for (Goal goal : Goal.values()) {
      assertNotEquals(
          RobotState.CLEARING,
          Transitions.next(from, goal, cond()),
          "CLEARING must only be entered from SHOOTING/PASSING, got it from " + from + "+" + goal);
    }
  }

  @Test
  void manualStaysManualInsideTransitions() {
    // T20/T21 are the Superstructure's mode layer; the pure machine never exits MANUAL itself.
    for (Goal goal : Goal.values()) {
      assertEquals(RobotState.MANUAL, Transitions.next(RobotState.MANUAL, goal, gateOpen()));
    }
  }

  // ── Return-to-rest + preemption ──

  @ParameterizedTest
  @EnumSource(
      value = RobotState.class,
      names = {"IDLE", "INTAKING", "SPINNING_UP", "PASS_AIMING", "PASS_SPINNING_UP", "UNJAMMING"})
  void everyStateReturnsToRestWhenGoalIdle(RobotState from) {
    assertEquals(RobotState.IDLE, Transitions.next(from, Goal.IDLE, cond()));
  }

  @Test
  void clearingReturnsToRestOnlyAfterElapsed() {
    assertEquals(RobotState.CLEARING, Transitions.next(RobotState.CLEARING, Goal.IDLE, cond()));
    assertEquals(
        RobotState.IDLE,
        Transitions.next(RobotState.CLEARING, Goal.IDLE, with(false, false, false, false, false, true)));
  }

  @Test
  void goalPreemptionWinsImmediately() {
    // A new goal replans from the current state in the same tick (UNJAM mid-volley).
    assertEquals(RobotState.UNJAMMING, Transitions.next(RobotState.SHOOTING, Goal.UNJAM, gateOpen()));
    assertEquals(RobotState.UNJAMMING, Transitions.next(RobotState.PASSING, Goal.UNJAM, gateOpen()));
  }
}
