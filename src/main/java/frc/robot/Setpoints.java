package frc.robot;

public class Setpoints {
    public static final double kFeederStationArm = 38.5;
    public static final double kFeederStationElevator = 0;
    public static final double kLevel1Arm = 6;
    public static final double kLevel1Elevator = 0;    
    public static final double kLevel2Arm = 3;
    public static final double kLevel2Elevator = 64;
    public static final double kLevel3Arm = 3.8;
    public static final double kLevel3Elevator = 153;
    public static final double kLevel4Arm = 39.6;
    public static final double kLevel4Elevator = 184;

    public enum Setpoint {
        kFeederStation, kLevel1, kLevel2, kLevel3, kLevel4
    }
}

