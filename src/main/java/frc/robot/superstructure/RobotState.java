package frc.robot.superstructure;

/** The scoring-chain state. See docs/superstructure.md for the diagram (kept in lockstep). */
public enum RobotState {
  IDLE, // turret tracks hub (tracking IS idle on this robot), shooter off, hood holds
  INTAKING, // IDLE + intake deployed/rolling + vertical feed at 26.25 RPS
  SPINNING_UP, // SHOOT requested: shooter+hood track interp tables, feed gate closed
  SHOOTING, // aimed && at speed: all indexer stages feed
  CLEARING, // [ATOMIC] shot released: shooter stops, all indexers reverse FULL for kClearingSeconds
  PASS_AIMING, // PASS modifier: turret aims along passing-tag normal, shooter off
  PASS_SPINNING_UP, // SHOOT+pass: hood 0.067, shooter 95 RPS, feed gate closed
  PASSING, // at speed: feed
  UNJAMMING, // indexers reversed 50%, shooter forward at max
  MANUAL // superstructure commands nothing; operator drives mechanisms directly
}
