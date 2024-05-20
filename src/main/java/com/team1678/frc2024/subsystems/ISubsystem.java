package com.team1678.frc2024.subsystems;

import com.team1678.frc2024.loops.ILooper;

/**
 * The Subsystem interface, which serves as a basic framework for all robot subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and functions get the
 * instance of the drivetrain and act accordingly. Subsystems are also a state machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each Subsystem also is responsible for
 * instantiating all member components at the start of the match.
 */
public interface ISubsystem {

    void readPeriodicInputs();

    void writePeriodicOutputs();

    void outputTelemetry();

    void stop();

    void zeroSensors();

    void registerLooper(ILooper looper);

    boolean hasEmergency = false;

}
