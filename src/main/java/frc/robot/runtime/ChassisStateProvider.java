package frc.robot.runtime;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The slice of the drivetrain the scoring axis reads: the continuous Pigeon yaw (the turret gyro
 * feedforward signal, W3 — does NOT wrap at ±180) and the robot-relative chassis speeds (the
 * shoot-while-moving correction input). The {@code Drive} adapter satisfies this directly; keeping
 * it an interface lets the interpreter be tested without constructing the swerve drivetrain.
 */
public interface ChassisStateProvider {
  double continuousYawDeg();

  ChassisSpeeds robotRelativeSpeeds();
}
