package com.team1678.frc2024;

public class Settings {

    public static final boolean kIsUsingCompBot = true;

    public static final boolean kLogToFile = false;

    public enum RobotMode {
        REAL,
        SIM,
        REPLAY
    }

    public static final RobotMode kRobotMode = RobotMode.SIM;

    public static final boolean kResetMotorControllers = false;

}
