package frc.robot.autonomy.bt;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Factories for the two leaf kinds: a condition (guard) and an action. Keeps the tree readable. */
public final class Leaf {
  private Leaf() {}

  /** A guard: SUCCESS when the predicate is true this tick, FAILURE otherwise. Never RUNNING. */
  public static Node condition(BooleanSupplier predicate) {
    return () -> predicate.getAsBoolean() ? Status.SUCCESS : Status.FAILURE;
  }

  /** An action that computes its own {@link Status} each tick. */
  public static Node action(Supplier<Status> body) {
    return body::get;
  }

  /** A fire-and-forget action that always reports SUCCESS (e.g. publish a request). */
  public static Node run(Runnable body) {
    return () -> {
      body.run();
      return Status.SUCCESS;
    };
  }
}
