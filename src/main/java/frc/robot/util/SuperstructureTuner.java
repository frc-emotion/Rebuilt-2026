package frc.robot.util;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * Unified dashboard tuner. Every tunable constant lives here.
 * Open Elastic → /Tuning/ → edit any value → it hot-applies to the motor
 * and persists across reboots via Preferences.
 *
 * <p>Call {@link #checkForChanges()} from Robot.robotPeriodic().
 */
@Logged
public class SuperstructureTuner {

    // ========================================
    // TURRET
    // ========================================
    public final TunableNumber turretKP = new TunableNumber("Turret/kP", 0.01);
    public final TunableNumber turretKI = new TunableNumber("Turret/kI", 0.0);
    public final TunableNumber turretKD = new TunableNumber("Turret/kD", 0.5);
    public final TunableNumber turretKS = new TunableNumber("Turret/kS", 0.5);
    public final TunableNumber turretKV = new TunableNumber("Turret/kV", 0.12);
    public final TunableNumber turretMMCruise = new TunableNumber("Turret/MotionMagic/CruiseVelocity", 2.0);
    public final TunableNumber turretMMAcc = new TunableNumber("Turret/MotionMagic/Acceleration", 4.0);
    public final TunableNumber turretMMJerk = new TunableNumber("Turret/MotionMagic/Jerk", 40.0);
    public final TunableNumber turretManualVolts = new TunableNumber("Turret/ManualVoltageScale", 3.0);

    // ========================================
    // HOOD
    // ========================================
    public final TunableNumber hoodKP = new TunableNumber("Hood/kP", 0.01);
    public final TunableNumber hoodKI = new TunableNumber("Hood/kI", 0.0);
    public final TunableNumber hoodKD = new TunableNumber("Hood/kD", 0.0);
    public final TunableNumber hoodKS = new TunableNumber("Hood/kS", 0.5);
    public final TunableNumber hoodKV = new TunableNumber("Hood/kV", 0.0);
    public final TunableNumber hoodMMCruise = new TunableNumber("Hood/MotionMagic/CruiseVelocity", 1.0);
    public final TunableNumber hoodMMAcc = new TunableNumber("Hood/MotionMagic/Acceleration", 2.0);
    public final TunableNumber hoodMMJerk = new TunableNumber("Hood/MotionMagic/Jerk", 20.0);
    public final TunableNumber hoodManualVolts = new TunableNumber("Hood/ManualVoltageScale", 3.0);

    // ========================================
    // SHOOTER
    // ========================================
    public final TunableNumber shooterKP = new TunableNumber("Shooter/kP", 0.1);
    public final TunableNumber shooterKI = new TunableNumber("Shooter/kI", 0.0);
    public final TunableNumber shooterKD = new TunableNumber("Shooter/kD", 0.0);
    public final TunableNumber shooterKS = new TunableNumber("Shooter/kS", 0.0);
    public final TunableNumber shooterKV = new TunableNumber("Shooter/kV", 0.0);
    public final TunableNumber shooterTolerance = new TunableNumber("Shooter/Tolerance", 0.5);

    // ========================================
    // INTAKE
    // ========================================
    public final TunableNumber intakeKP = new TunableNumber("Intake/kP", 25.0);
    public final TunableNumber intakeKI = new TunableNumber("Intake/kI", 0.0);
    public final TunableNumber intakeKD = new TunableNumber("Intake/kD", 0.0);
    public final TunableNumber intakeKS = new TunableNumber("Intake/kS", 0.5);
    public final TunableNumber intakeKG = new TunableNumber("Intake/kG", 0.53);
    public final TunableNumber intakeMMCruise = new TunableNumber("Intake/MotionMagic/CruiseVelocity", 2.0);
    public final TunableNumber intakeMMAcc = new TunableNumber("Intake/MotionMagic/Acceleration", 4.0);
    public final TunableNumber intakeMMJerk = new TunableNumber("Intake/MotionMagic/Jerk", 40.0);
    public final TunableNumber intakeOutAngleDeg = new TunableNumber("Intake/OutAngleDegrees", 85.0);
    public final TunableNumber intakeRollerSpeed = new TunableNumber("Intake/RollerSpeedRPS", 40.0);

    // ========================================
    // INDEXER SPEEDS
    // ========================================
    public final TunableNumber indexerHorizontalSpeed = new TunableNumber("Indexer/HorizontalSpeed", 30.0);
    public final TunableNumber indexerVerticalSpeed = new TunableNumber("Indexer/VerticalSpeed", 30.0);
    public final TunableNumber indexerUpwardSpeed = new TunableNumber("Indexer/UpwardSpeed", 100.0);

    // ========================================
    // AUTO-AIM
    // ========================================
    public final TunableNumber aimVisionKP = new TunableNumber("AutoAim/VisionKP", 0.003);
    public final TunableNumber aimGyroFF = new TunableNumber("AutoAim/GyroFF", 0.15);
    public final TunableNumber aimTurretTolerance = new TunableNumber("AutoAim/TurretToleranceRot", 0.05);

    // ========================================
    // INTERPOLATION TABLE: Distance (m) → Flywheel RPM
    // ========================================
    public final TunableNumber flywheel1m = new TunableNumber("ShotTable/Flywheel/1m_RPM", 2000.0);
    public final TunableNumber flywheel2m = new TunableNumber("ShotTable/Flywheel/2m_RPM", 2500.0);
    public final TunableNumber flywheel3m = new TunableNumber("ShotTable/Flywheel/3m_RPM", 3000.0);
    public final TunableNumber flywheel4m = new TunableNumber("ShotTable/Flywheel/4m_RPM", 3500.0);
    public final TunableNumber flywheel5m = new TunableNumber("ShotTable/Flywheel/5m_RPM", 4000.0);
    public final TunableNumber flywheel6m = new TunableNumber("ShotTable/Flywheel/6m_RPM", 4500.0);

    // ========================================
    // INTERPOLATION TABLE: Distance (m) → Hood Angle (degrees)
    // ========================================
    public final TunableNumber hood1m = new TunableNumber("ShotTable/Hood/1m_deg", 60.0);
    public final TunableNumber hood2m = new TunableNumber("ShotTable/Hood/2m_deg", 50.0);
    public final TunableNumber hood3m = new TunableNumber("ShotTable/Hood/3m_deg", 45.0);
    public final TunableNumber hood4m = new TunableNumber("ShotTable/Hood/4m_deg", 40.0);
    public final TunableNumber hood5m = new TunableNumber("ShotTable/Hood/5m_deg", 35.0);
    public final TunableNumber hood6m = new TunableNumber("ShotTable/Hood/6m_deg", 30.0);

    // Subsystem references for hot-reload
    private Turret turret;
    private Hood hood;
    private Shooter shooter;
    private Intake intake;

    /**
     * Wire subsystems for hot-reload. Pass null for any that aren't enabled.
     */
    public void setSubsystems(Turret turret, Hood hood, Shooter shooter, Intake intake) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.intake = intake;
    }

    /**
     * Call this every robot periodic cycle. Checks if any value changed in Elastic
     * and hot-applies the new config to the motor controller. Only sends CAN frames
     * when a value actually changes — zero overhead otherwise.
     */
    public void checkForChanges() {
        checkTurretPID();
        checkTurretMotionMagic();
        checkHoodPID();
        checkHoodMotionMagic();
        checkShooterPID();
        checkIntakePID();
        checkIntakeMotionMagic();
    }

    /**
     * Rebuild the flywheel interpolation table from current dashboard values.
     * Call this when shot table values change or pass to TurretAimingCalculator.
     */
    public InterpolatingDoubleTreeMap buildFlywheelTable() {
        InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
        table.put(1.0, flywheel1m.get());
        table.put(2.0, flywheel2m.get());
        table.put(3.0, flywheel3m.get());
        table.put(4.0, flywheel4m.get());
        table.put(5.0, flywheel5m.get());
        table.put(6.0, flywheel6m.get());
        return table;
    }

    /**
     * Rebuild the hood angle interpolation table from current dashboard values.
     */
    public InterpolatingDoubleTreeMap buildHoodTable() {
        InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
        table.put(1.0, hood1m.get());
        table.put(2.0, hood2m.get());
        table.put(3.0, hood3m.get());
        table.put(4.0, hood4m.get());
        table.put(5.0, hood5m.get());
        table.put(6.0, hood6m.get());
        return table;
    }

    // ========================================
    // HOT-RELOAD HELPERS
    // ========================================

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
