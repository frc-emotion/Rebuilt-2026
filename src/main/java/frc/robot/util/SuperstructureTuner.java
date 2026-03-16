package frc.robot.util;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.epilogue.Logged;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * Live PID and MotionMagic tuner. Edit values in Elastic under /Tuning/,
 * changes hot-apply to motors and persist across reboots.
 *
 * <p>Call {@link #checkForChanges()} from Robot.robotPeriodic().
 */
@Logged
public class SuperstructureTuner {

    // Turret PID — defaults MUST match TurretConstants.TURRET_CONFIG
    public final TunableNumber turretKP = new TunableNumber("Turret/kP", 20.0);
    public final TunableNumber turretKI = new TunableNumber("Turret/kI", 7.5);
    public final TunableNumber turretKD = new TunableNumber("Turret/kD", 1.52658);
    public final TunableNumber turretKS = new TunableNumber("Turret/kS", 0.0);
    public final TunableNumber turretKV = new TunableNumber("Turret/kV", 0.0);

    // Turret MotionMagic
    public final TunableNumber turretMMCruise = new TunableNumber("Turret/MotionMagic/CruiseVelocity", 1.0);
    public final TunableNumber turretMMAcc = new TunableNumber("Turret/MotionMagic/Acceleration", 4.0);
    public final TunableNumber turretMMJerk = new TunableNumber("Turret/MotionMagic/Jerk", 40.0);

    // Hood PID — defaults MUST match TurretConstants.HOOD_CONFIG
    public final TunableNumber hoodKP = new TunableNumber("Hood/kP", 100.0);
    public final TunableNumber hoodKI = new TunableNumber("Hood/kI", 50.0);
    public final TunableNumber hoodKD = new TunableNumber("Hood/kD", 0.0);
    public final TunableNumber hoodKS = new TunableNumber("Hood/kS", 0.0);
    public final TunableNumber hoodKV = new TunableNumber("Hood/kV", 0.0);

    // Hood MotionMagic
    public final TunableNumber hoodMMCruise = new TunableNumber("Hood/MotionMagic/CruiseVelocity", 1.0);
    public final TunableNumber hoodMMAcc = new TunableNumber("Hood/MotionMagic/Acceleration", 2.0);
    public final TunableNumber hoodMMJerk = new TunableNumber("Hood/MotionMagic/Jerk", 20.0);

    // Shooter PID — defaults MUST match TurretConstants.SHOOTER_CONFIG
    public final TunableNumber shooterKP = new TunableNumber("Shooter/kP", 0.1);
    public final TunableNumber shooterKI = new TunableNumber("Shooter/kI", 0.0);
    public final TunableNumber shooterKD = new TunableNumber("Shooter/kD", 0.0);
    public final TunableNumber shooterKS = new TunableNumber("Shooter/kS", 0.0);
    public final TunableNumber shooterKV = new TunableNumber("Shooter/kV", 0.0);

    // Intake PID — defaults MUST match IntakeConstants.INTAKE_CONFIG
    public final TunableNumber intakeKP = new TunableNumber("Intake/kP", 13.0);
    public final TunableNumber intakeKI = new TunableNumber("Intake/kI", 0.0);
    public final TunableNumber intakeKD = new TunableNumber("Intake/kD", 0.0);
    public final TunableNumber intakeKS = new TunableNumber("Intake/kS", 0.0);
    public final TunableNumber intakeKG = new TunableNumber("Intake/kG", 0.0);

    // Intake MotionMagic
    public final TunableNumber intakeMMCruise = new TunableNumber("Intake/MotionMagic/CruiseVelocity", 2.0);
    public final TunableNumber intakeMMAcc = new TunableNumber("Intake/MotionMagic/Acceleration", 4.0);
    public final TunableNumber intakeMMJerk = new TunableNumber("Intake/MotionMagic/Jerk", 40.0);

    private Turret turret;
    private Hood hood;
    private Shooter shooter;
    private Intake intake;

    public void setSubsystems(Turret turret, Hood hood, Shooter shooter, Intake intake) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.intake = intake;
    }

    /** Check all tunables and hot-apply changes. Zero CAN overhead when nothing changed. */
    public void checkForChanges() {
        checkTurretPID();
        checkTurretMotionMagic();
        checkHoodPID();
        checkHoodMotionMagic();
        checkShooterPID();
        checkIntakePID();
        checkIntakeMotionMagic();
    }

    private void checkTurretPID() {
        if (turret == null) return;
        if (turretKP.hasChanged() || turretKI.hasChanged() || turretKD.hasChanged()
                || turretKS.hasChanged() || turretKV.hasChanged()) {
            turret.getTurretMotor().getConfigurator().apply(
                    new Slot0Configs()
                            .withKP(turretKP.get())
                            .withKI(turretKI.get())
                            .withKD(turretKD.get())
                            .withKS(turretKS.get())
                            .withKV(turretKV.get()),
                    0.05);
        }
    }

    private void checkTurretMotionMagic() {
        if (turret == null) return;
        if (turretMMCruise.hasChanged() || turretMMAcc.hasChanged() || turretMMJerk.hasChanged()) {
            turret.getTurretMotor().getConfigurator().apply(
                    new MotionMagicConfigs()
                            .withMotionMagicCruiseVelocity(turretMMCruise.get())
                            .withMotionMagicAcceleration(turretMMAcc.get())
                            .withMotionMagicJerk(turretMMJerk.get()),
                    0.05);
        }
    }

    private void checkHoodPID() {
        if (hood == null) return;
        if (hoodKP.hasChanged() || hoodKI.hasChanged() || hoodKD.hasChanged()
                || hoodKS.hasChanged() || hoodKV.hasChanged()) {
            hood.getHoodMotor().getConfigurator().apply(
                    new Slot0Configs()
                            .withKP(hoodKP.get())
                            .withKI(hoodKI.get())
                            .withKD(hoodKD.get())
                            .withKS(hoodKS.get())
                            .withKV(hoodKV.get()),
                    0.05);
        }
    }

    private void checkHoodMotionMagic() {
        if (hood == null) return;
        if (hoodMMCruise.hasChanged() || hoodMMAcc.hasChanged() || hoodMMJerk.hasChanged()) {
            hood.getHoodMotor().getConfigurator().apply(
                    new MotionMagicConfigs()
                            .withMotionMagicCruiseVelocity(hoodMMCruise.get())
                            .withMotionMagicAcceleration(hoodMMAcc.get())
                            .withMotionMagicJerk(hoodMMJerk.get()),
                    0.05);
        }
    }

    private void checkShooterPID() {
        if (shooter == null) return;
        if (shooterKP.hasChanged() || shooterKI.hasChanged() || shooterKD.hasChanged()
                || shooterKS.hasChanged() || shooterKV.hasChanged()) {
            shooter.getShooterMotor().getConfigurator().apply(
                    new Slot0Configs()
                            .withKP(shooterKP.get())
                            .withKI(shooterKI.get())
                            .withKD(shooterKD.get())
                            .withKS(shooterKS.get())
                            .withKV(shooterKV.get()),
                    0.05);
        }
    }

    private void checkIntakePID() {
        if (intake == null) return;
        if (intakeKP.hasChanged() || intakeKI.hasChanged() || intakeKD.hasChanged()
                || intakeKS.hasChanged() || intakeKG.hasChanged()) {
            intake.getIntakeMotor().getConfigurator().apply(
                    new Slot0Configs()
                            .withKP(intakeKP.get())
                            .withKI(intakeKI.get())
                            .withKD(intakeKD.get())
                            .withKS(intakeKS.get())
                            .withKG(intakeKG.get()),
                    0.05);
        }
    }

    private void checkIntakeMotionMagic() {
        if (intake == null) return;
        if (intakeMMCruise.hasChanged() || intakeMMAcc.hasChanged() || intakeMMJerk.hasChanged()) {
            intake.getIntakeMotor().getConfigurator().apply(
                    new MotionMagicConfigs()
                            .withMotionMagicCruiseVelocity(intakeMMCruise.get())
                            .withMotionMagicAcceleration(intakeMMAcc.get())
                            .withMotionMagicJerk(intakeMMJerk.get()),
                    0.05);
        }
    }
}
