package frc.robot.Constants;

import com.ctre.phoenix6.CANBus;

public class Gen {
        public static final boolean enableVision = true;
        public static final boolean enableIntake = true;
        public static final boolean enableIndexer = true;
        public static final boolean enableTurret = true;
        public static final boolean enableHood = true;
        public static final boolean enableShooter = true;

        public static final CANBus mechanismBus = new CANBus("mechanisms");

        public static final int driverPort = 0;
        public static final int operatorPort = 1;

}
