package frc.robot.runtime.reflex;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import frc.robot.runtime.reflex.ScoringSequencer.Conditions;
import frc.robot.runtime.reflex.ScoringSequencer.Goal;
import frc.robot.runtime.reflex.ScoringSequencer.Phase;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Every legal transition, key illegal-transition rejections, atomic completion under preemption,
 * and return-to-rest. IDs (T#) match REFACTOR_DESIGN.md §2 and docs/skills.md.
 */
class ScoringSequencerTest {

  // Conditions builder: everything false/closed by default, opt in per test.
  private static Conditions cond() {
    return new Conditions(false, false, false, false, false, false);
  }

  private static Conditions gateOpen() {
    return new Conditions(false, true, true, true, false, false);
  }

  private static Conditions with(
      boolean pass,
      boolean aimed,
      boolean atSpeed,
      boolean nonZero,
      boolean intake,
      boolean cleared) {
    return new Conditions(pass, aimed, atSpeed, nonZero, intake, cleared);
  }

  // ── Rest states (T0–T4) ──

  @Test
  void idleToIntakingOnToggle() {
    assertEquals(
        Phase.INTAKING,
        ScoringSequencer.next(
            Phase.IDLE, Goal.IDLE, with(false, false, false, false, true, false)));
  }

  @Test
  void intakingToIdleOnToggle() {
    assertEquals(Phase.IDLE, ScoringSequencer.next(Phase.INTAKING, Goal.IDLE, cond()));
  }

  @Test
  void idleToSpinningUpOnShoot() {
    assertEquals(Phase.SPINNING_UP, ScoringSequencer.next(Phase.IDLE, Goal.SHOOT, cond()));
  }

  @Test
  void intakingToSpinningUpKeepsIntakeDeployed() {
    // The intake axis never blocks a scoring request; deployment carries via the condition.
    assertEquals(
        Phase.SPINNING_UP,
        ScoringSequencer.next(
            Phase.INTAKING, Goal.SHOOT, with(false, false, false, false, true, false)));
  }

  @Test
  void idleToPassSpinningUpWithPassSelected() {
    assertEquals(
        Phase.PASS_SPINNING_UP,
        ScoringSequencer.next(
            Phase.IDLE, Goal.SHOOT, with(true, false, false, false, false, false)));
  }

  @Test
  void idleToPassAimingOnPass() {
    assertEquals(Phase.PASS_AIMING, ScoringSequencer.next(Phase.IDLE, Goal.PASS, cond()));
  }

  @Test
  void idleToUnjamming() {
    assertEquals(Phase.UNJAMMING, ScoringSequencer.next(Phase.IDLE, Goal.UNJAM, cond()));
  }

  // ── SPINNING_UP (T5–T7) ──

  @Test
  void spinningUpHoldsUntilFullGate() {
    // The W12 regression test: aimed alone, speed alone, or a ZERO setpoint must never fire.
    assertEquals(
        Phase.SPINNING_UP,
        ScoringSequencer.next(
            Phase.SPINNING_UP, Goal.SHOOT, with(false, true, false, false, false, false)));
    assertEquals(
        Phase.SPINNING_UP,
        ScoringSequencer.next(
            Phase.SPINNING_UP, Goal.SHOOT, with(false, false, true, true, false, false)));
    assertEquals(
        Phase.SPINNING_UP,
        ScoringSequencer.next(
            Phase.SPINNING_UP, Goal.SHOOT, with(false, true, true, false, false, false)));
  }

  @Test
  void spinningUpToShootingWhenGateSatisfied() {
    assertEquals(Phase.SHOOTING, ScoringSequencer.next(Phase.SPINNING_UP, Goal.SHOOT, gateOpen()));
  }

  @Test
  void spinningUpReleaseSkipsClearing() {
    // No balls committed yet — releasing the trigger goes straight to rest, no clear (T7).
    assertEquals(Phase.IDLE, ScoringSequencer.next(Phase.SPINNING_UP, Goal.IDLE, cond()));
  }

  @Test
  void spinningUpToPassChainOnLb() {
    assertEquals(
        Phase.PASS_SPINNING_UP,
        ScoringSequencer.next(
            Phase.SPINNING_UP, Goal.SHOOT, with(true, false, false, false, false, false)));
  }

  // ── SHOOTING (T8–T10u) ──

  @Test
  void shootingReclosesOnSpeedDroop() {
    assertEquals(
        Phase.SPINNING_UP,
        ScoringSequencer.next(
            Phase.SHOOTING, Goal.SHOOT, with(false, true, false, true, false, false)));
  }

  @Test
  void shootingReclosesOnAimLoss() {
    assertEquals(
        Phase.SPINNING_UP,
        ScoringSequencer.next(
            Phase.SHOOTING, Goal.SHOOT, with(false, false, true, true, false, false)));
  }

  @Test
  void shootingReleaseEntersClearing() {
    assertEquals(Phase.CLEARING, ScoringSequencer.next(Phase.SHOOTING, Goal.IDLE, gateOpen()));
    assertEquals(Phase.CLEARING, ScoringSequencer.next(Phase.SHOOTING, Goal.PASS, gateOpen()));
  }

  @Test
  void unjamPreemptsShooting() {
    assertEquals(Phase.UNJAMMING, ScoringSequencer.next(Phase.SHOOTING, Goal.UNJAM, gateOpen()));
  }

  // ── CLEARING (T22–T24, the atomic transition) ──

  @Test
  void clearingDefersIdleAndPassGoalsUntilElapsed() {
    // Atomic completion under preemption: goal flaps IDLE/PASS mid-clear, state stays CLEARING.
    assertEquals(Phase.CLEARING, ScoringSequencer.next(Phase.CLEARING, Goal.IDLE, cond()));
    assertEquals(Phase.CLEARING, ScoringSequencer.next(Phase.CLEARING, Goal.PASS, cond()));
  }

  @Test
  void clearingExitsToIdleOrIntakingPerIntakeToggle() {
    assertEquals(
        Phase.IDLE,
        ScoringSequencer.next(
            Phase.CLEARING, Goal.IDLE, with(false, false, false, false, false, true)));
    assertEquals(
        Phase.INTAKING,
        ScoringSequencer.next(
            Phase.CLEARING, Goal.IDLE, with(false, false, false, false, true, true)));
  }

  @Test
  void clearingExitsToPassAimingWhenLbHeld() {
    assertEquals(
        Phase.PASS_AIMING,
        ScoringSequencer.next(
            Phase.CLEARING, Goal.PASS, with(true, false, false, false, false, true)));
  }

  @Test
  void clearingReentersShootChain() {
    assertEquals(Phase.SPINNING_UP, ScoringSequencer.next(Phase.CLEARING, Goal.SHOOT, cond()));
    assertEquals(
        Phase.PASS_SPINNING_UP,
        ScoringSequencer.next(
            Phase.CLEARING, Goal.SHOOT, with(true, false, false, false, false, false)));
  }

  @Test
  void unjamPreemptsClearing() {
    assertEquals(Phase.UNJAMMING, ScoringSequencer.next(Phase.CLEARING, Goal.UNJAM, cond()));
  }

  // ── Pass chain (T11–T18u) ──

  @Test
  void passAimingToPassSpinningUpOnShoot() {
    assertEquals(
        Phase.PASS_SPINNING_UP,
        ScoringSequencer.next(
            Phase.PASS_AIMING, Goal.SHOOT, with(true, false, false, false, false, false)));
  }

  @Test
  void passAimingReturnsToRest() {
    assertEquals(Phase.IDLE, ScoringSequencer.next(Phase.PASS_AIMING, Goal.IDLE, cond()));
  }

  @Test
  void passChainRequiresOnlySpeed() {
    // Legacy quirk, preserved: passing never consults aimed.
    assertEquals(
        Phase.PASSING,
        ScoringSequencer.next(
            Phase.PASS_SPINNING_UP, Goal.SHOOT, with(true, false, true, true, false, false)));
  }

  @Test
  void lbReleaseMidPassSpinFallsBackToHubChain() {
    assertEquals(
        Phase.SPINNING_UP,
        ScoringSequencer.next(
            Phase.PASS_SPINNING_UP, Goal.SHOOT, with(false, false, true, true, false, false)));
  }

  @Test
  void passingReclosesOnSpeedDroop() {
    assertEquals(
        Phase.PASS_SPINNING_UP,
        ScoringSequencer.next(
            Phase.PASSING, Goal.SHOOT, with(true, false, false, false, false, false)));
  }

  @Test
  void passingReleaseEntersClearing() {
    assertEquals(
        Phase.CLEARING,
        ScoringSequencer.next(
            Phase.PASSING, Goal.PASS, with(true, false, true, true, false, false)));
  }

  // ── Remaining direct row coverage (gate-report gaps) ──

  @Test
  void shootingToPassChainOnLbMidVolley() { // T9
    assertEquals(
        Phase.PASS_SPINNING_UP,
        ScoringSequencer.next(
            Phase.SHOOTING, Goal.SHOOT, with(true, true, true, true, false, false)));
  }

  @Test
  void passAimingToUnjamming() { // T13
    assertEquals(Phase.UNJAMMING, ScoringSequencer.next(Phase.PASS_AIMING, Goal.UNJAM, cond()));
  }

  @Test
  void intakingSharesAllOutboundTransitions() { // T2/T3/T4 from INTAKING
    Conditions deployed = with(true, false, false, false, true, false);
    assertEquals(
        Phase.PASS_SPINNING_UP, ScoringSequencer.next(Phase.INTAKING, Goal.SHOOT, deployed));
    assertEquals(Phase.PASS_AIMING, ScoringSequencer.next(Phase.INTAKING, Goal.PASS, deployed));
    assertEquals(Phase.UNJAMMING, ScoringSequencer.next(Phase.INTAKING, Goal.UNJAM, deployed));
  }

  @Test
  void spinUpExitsResolvePerGoal() { // T7 PASS/UNJAM variants
    assertEquals(Phase.PASS_AIMING, ScoringSequencer.next(Phase.SPINNING_UP, Goal.PASS, cond()));
    assertEquals(Phase.UNJAMMING, ScoringSequencer.next(Phase.SPINNING_UP, Goal.UNJAM, cond()));
  }

  @Test
  void passSpinUpExitsResolvePerGoal() { // T16 PASS/UNJAM variants
    assertEquals(
        Phase.PASS_AIMING, ScoringSequencer.next(Phase.PASS_SPINNING_UP, Goal.PASS, cond()));
    assertEquals(
        Phase.UNJAMMING, ScoringSequencer.next(Phase.PASS_SPINNING_UP, Goal.UNJAM, cond()));
  }

  @Test
  void passingIdleReleaseAlsoEntersClearing() { // T18 with goal IDLE
    assertEquals(Phase.CLEARING, ScoringSequencer.next(Phase.PASSING, Goal.IDLE, gateOpen()));
  }

  @Test
  void passAimingReturnsToIntakingWhenDeployed() { // T12 landing on INTAKING
    assertEquals(
        Phase.INTAKING,
        ScoringSequencer.next(
            Phase.PASS_AIMING, Goal.IDLE, with(false, false, false, false, true, false)));
  }

  @Test
  void passingLbReleaseFallsBackToHubChain() { // T15 mirror (documented in diagram)
    assertEquals(
        Phase.SPINNING_UP,
        ScoringSequencer.next(
            Phase.PASSING, Goal.SHOOT, with(false, false, true, true, false, false)));
  }

  // ── UNJAMMING (T19) ──

  @Test
  void unjammingReturnsToRestOnRelease() {
    assertEquals(Phase.IDLE, ScoringSequencer.next(Phase.UNJAMMING, Goal.IDLE, cond()));
    assertEquals(
        Phase.INTAKING,
        ScoringSequencer.next(
            Phase.UNJAMMING, Goal.IDLE, with(false, false, false, false, true, false)));
    assertEquals(
        Phase.PASS_AIMING,
        ScoringSequencer.next(
            Phase.UNJAMMING, Goal.PASS, with(true, false, false, false, false, false)));
  }

  // ── Illegal-transition rejections ──

  @Test
  void shootingUnreachableFromIdleDirectly() {
    // Even with the full gate satisfied, IDLE+SHOOT lands in SPINNING_UP, never SHOOTING.
    assertNotEquals(Phase.SHOOTING, ScoringSequencer.next(Phase.IDLE, Goal.SHOOT, gateOpen()));
  }

  @Test
  void passingUnreachableWithoutSpeed() {
    assertNotEquals(
        Phase.PASSING,
        ScoringSequencer.next(
            Phase.PASS_SPINNING_UP, Goal.SHOOT, with(true, true, false, true, false, false)));
  }

  @ParameterizedTest
  @EnumSource(
      value = Phase.class,
      names = {"IDLE", "INTAKING", "SPINNING_UP", "PASS_AIMING", "PASS_SPINNING_UP", "UNJAMMING"})
  void clearingUnreachableExceptFromShootingOrPassing(Phase from) {
    for (Goal goal : Goal.values()) {
      assertNotEquals(
          Phase.CLEARING,
          ScoringSequencer.next(from, goal, cond()),
          "CLEARING must only be entered from SHOOTING/PASSING, got it from " + from + "+" + goal);
    }
  }

  @Test
  void manualStaysManualInsideTransitions() {
    // T20/T21 are the Superstructure's mode layer; the pure machine never exits MANUAL itself.
    for (Goal goal : Goal.values()) {
      assertEquals(Phase.MANUAL, ScoringSequencer.next(Phase.MANUAL, goal, gateOpen()));
    }
  }

  // ── Return-to-rest + preemption ──

  @ParameterizedTest
  @EnumSource(
      value = Phase.class,
      names = {"IDLE", "INTAKING", "SPINNING_UP", "PASS_AIMING", "PASS_SPINNING_UP", "UNJAMMING"})
  void everyStateReturnsToRestWhenGoalIdle(Phase from) {
    assertEquals(Phase.IDLE, ScoringSequencer.next(from, Goal.IDLE, cond()));
  }

  @Test
  void clearingReturnsToRestOnlyAfterElapsed() {
    assertEquals(Phase.CLEARING, ScoringSequencer.next(Phase.CLEARING, Goal.IDLE, cond()));
    assertEquals(
        Phase.IDLE,
        ScoringSequencer.next(
            Phase.CLEARING, Goal.IDLE, with(false, false, false, false, false, true)));
  }

  @Test
  void goalPreemptionWinsImmediately() {
    // A new goal replans from the current state in the same tick (UNJAM mid-volley).
    assertEquals(Phase.UNJAMMING, ScoringSequencer.next(Phase.SHOOTING, Goal.UNJAM, gateOpen()));
    assertEquals(Phase.UNJAMMING, ScoringSequencer.next(Phase.PASSING, Goal.UNJAM, gateOpen()));
  }
}
