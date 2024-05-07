package com.team1678.lib.util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;
import java.util.logging.Logger;

/**
 * Tracks start-up and caught crash events, logging them to a file which doesn't roll over
 */
public class CrashTracker {
    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

    public static void logRobotStartup() {
        logMarker("Robot Startup");
    }

    public static void logRoboConstruction() {
        logMarker("Robot Construction");
    }

    public static void logRobotInit(){
        logMarker("Robot Init");
    }

    public static void logTeleopInit() {
        logMarker("Teleop Init");
    }

    public static void logDisabledInit() {
        logMarker("Disabled Init");
    }

    public static void logThrowableCrash(Throwable throwable) {
        logMarker("Exception", throwable);
    }

    private static void logMarker(String mark) {
        logMarker(mark, null);
    }

    private static void logMarker(String mark, Throwable nullableException) {
        try (PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/crash_tracking.txt", true))) {
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date().toString());

            if (nullableException != null) {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }
            writer.println();
        } catch (IOException e) {
            Logger logger = Logger.getAnonymousLogger();
            logger.severe("An error with creating the crash tracking writer occurred: ");
            logger.severe(e.toString());
        }
    }
}
